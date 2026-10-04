from setuptools import setup,find_packages
from glob import glob
import os
name='drone_description'
data=[('share/ament_index/resource_index/packages',['resource/'+name]),('share/'+name,['package.xml'])]
for folder in ('config','launch','worlds','urdf','rviz'):
    for root,dirs,files in os.walk(folder):
        data.append(('share/'+name+'/'+root,[os.path.join(root,f) for f in files]))
setup(name=name,version='0.1.0',packages=find_packages(),data_files=data,install_requires=['setuptools'],zip_safe=True,maintainer='MathTech',maintainer_email='maintainer@example.com',description='ApertureNav simulation module',license='MIT',tests_require=['pytest'],entry_points={'console_scripts':[]})
