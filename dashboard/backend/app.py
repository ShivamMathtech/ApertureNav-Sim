"""Local FastAPI dashboard service. APERTURE_MODE=ros uses actual ROS telemetry."""
import asyncio
from contextlib import asynccontextmanager, suppress
import copy
import json
import os
from pathlib import Path
import time
from typing import Literal
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, Response
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field
from aperturenav.config import load
from aperturenav.simulation import Simulation

ROOT=Path(__file__).resolve().parents[2]
CONFIG=Path(os.environ.get('APERTURE_CONFIG',str(ROOT/'config/default.yaml')))
MODE=os.environ.get('APERTURE_MODE','demo')
TERMINAL=('COMPLETE','ABORT','EMERGENCY_HOVER')

class Service:
    def __init__(self):
        self.sim=Simulation(load(CONFIG)); self.saved=None; self.ros=None
        if MODE=='ros':
            from .ros_adapter import RosAdapter
            self.ros=RosAdapter()
    def snapshot(self):
        if self.ros: return self.ros.snapshot()
        data=self.sim.snapshot()
        data.update({'paused':self.sim.paused,'events':self.sim.mission.events[-16:],
                     'summary':self.sim.summary(),'result_folder':str(self.saved) if self.saved else None})
        return data
    def command(self, action):
        if self.ros:
            self.ros.command(action);return
        s=self.sim
        if action=='start': s.mission.start(s.time)
        elif action=='pause': s.paused=True
        elif action=='resume': s.paused=False
        elif action=='reset':
            if s.history and not self.saved:self.saved=s.save(ROOT/'experiments/results')
            s.reset();self.saved=None
        elif action in ('abort','hover','estop'):
            if s.mission.state not in TERMINAL:
                s.mission.transition('ABORT' if action=='abort' else 'EMERGENCY_HOVER',f'operator {action}',s.time)
                s.velocity[:]=0;s.pid.reset()
        else: raise ValueError('Unsupported command')
    async def loop(self):
        while True:
            if not self.ros:
                s=self.sim
                if s.mission.state not in ('IDLE',*TERMINAL):
                    s.step()
                if s.mission.state in TERMINAL and self.saved is None and s.history:
                    self.saved=s.save(ROOT/'experiments/results')
            await asyncio.sleep(self.sim.c['dt'])

@asynccontextmanager
async def lifespan(app):
    app.state.service=Service()
    task=asyncio.create_task(app.state.service.loop())
    yield
    task.cancel()
    with suppress(asyncio.CancelledError):await task
    if app.state.service.ros:app.state.service.ros.close()

app=FastAPI(title='ApertureNav-Sim',version='0.1.0',lifespan=lifespan)
class Command(BaseModel):
    action:Literal['start','pause','resume','reset','abort','hover','estop']
class Scenario(BaseModel):
    width:float=Field(ge=.3,le=2.)
    height:float=Field(ge=.25,le=2.)
    yaw_degrees:float=Field(default=0,ge=-45,le=45)
    wind_std:float=Field(default=0,ge=0,le=.03)
    seed:int=Field(default=42,ge=0,le=2147483647)

@app.get('/api/health')
async def health():return {'status':'ok','mode':MODE}
@app.get('/api/telemetry')
async def telemetry():return app.state.service.snapshot()
@app.post('/api/command')
async def command(command:Command):
    try:app.state.service.command(command.action)
    except ValueError as error:raise HTTPException(409,str(error)) from error
    return {'accepted':True,'action':command.action}
@app.post('/api/scenario')
async def scenario(scenario:Scenario):
    svc=app.state.service
    if svc.ros:raise HTTPException(409,'ROS scenarios are configured with YAML and launch files')
    if svc.sim.mission.state!='IDLE':raise HTTPException(409,'Reset the mission before changing the scenario')
    c=copy.deepcopy(svc.sim.c)
    c['aperture'].update(width=scenario.width,height=scenario.height,yaw=scenario.yaw_degrees*3.141592653589793/180)
    c['environment']['wind_std']=scenario.wind_std;c['seed']=scenario.seed
    svc.sim=Simulation(c)
    return svc.snapshot()
@app.get('/api/results')
async def results():
    root=ROOT/'experiments/results'
    return [dict(id=p.parent.name,**json.loads(p.read_text())) for p in sorted(root.glob('*/summary.json'),reverse=True)[:100]]
@app.get('/api/results/{run_id}/{filename}')
async def download(run_id:str,filename:str):
    if not run_id.replace('_','').isdigit() or filename not in ('summary.json','telemetry.csv','events.csv','config.yaml'):
        raise HTTPException(404,'Unknown result')
    path=ROOT/'experiments/results'/run_id/filename
    if not path.is_file():raise HTTPException(404,'Result not found')
    return FileResponse(path,filename=filename)
@app.get('/api/camera.jpg')
async def camera():
    ros=app.state.service.ros
    if not ros or not ros.jpeg:raise HTTPException(404,'Live camera is available only in ROS mode')
    return Response(ros.jpeg,media_type='image/jpeg')
@app.websocket('/ws')
async def stream(ws:WebSocket):
    await ws.accept()
    try:
        while True:
            await ws.send_json(app.state.service.snapshot());await asyncio.sleep(.1)
    except (WebSocketDisconnect,RuntimeError):return

DIST=ROOT/'dashboard/frontend/dist'
if DIST.is_dir():app.mount('/',StaticFiles(directory=DIST,html=True),name='dashboard')
