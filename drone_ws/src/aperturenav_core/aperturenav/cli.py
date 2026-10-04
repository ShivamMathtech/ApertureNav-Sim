import argparse
import copy
import json
from pathlib import Path
from .config import load
from .simulation import Simulation

def main():
    parser = argparse.ArgumentParser(description='Offline aperture-navigation experiments')
    parser.add_argument('--config',default='config/default.yaml')
    parser.add_argument('--output',default='experiments/results')
    parser.add_argument('--benchmark',action='store_true')
    args = parser.parse_args()
    config = load(args.config)
    if args.benchmark:
        results=[]
        for width in (1.2,1.,.8,.6,.5,.45):
            c=copy.deepcopy(config);c['aperture']['width']=width
            sim=Simulation(c);summary=sim.run();summary['opening_width']=width
            summary['folder']=str(sim.save(args.output));results.append(summary)
        report={'successful_aperture_traversal_rate':sum(r['mission_success'] for r in results)/len(results),
                'source':'idealized geometry benchmark', 'runs':results}
        Path(args.output).mkdir(parents=True,exist_ok=True)
        (Path(args.output)/'benchmark.json').write_text(json.dumps(report,indent=2))
        print(json.dumps(report,indent=2))
    else:
        sim=Simulation(config);result=sim.run();result['folder']=str(sim.save(args.output))
        print(json.dumps(result,indent=2))
        return 0 if result['mission_success'] else 2

if __name__=='__main__':
    raise SystemExit(main())
