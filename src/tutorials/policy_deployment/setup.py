from setuptools import setup

package_name = 'lite6_policy_deployment_demos'

setup(
 name=package_name,
 version='0.0.0',
 packages=[package_name],
 data_files=[
     ('share/ament_index/resource_index/packages',
             ['resource/' + package_name]),
     ('share/' + package_name, ['package.xml']),
     ('share/' + package_name + '/launch', ['launch/policy.launch.py']),
     ('share/' + package_name + '/config', ['config/policy.yaml']),
   ],
 install_requires=['setuptools'],
 zip_safe=True,
 maintainer='Peter David Fagan',
 maintainer_email='peterdavidfagan@gmail.com',
 description='TODO: Package description',
 license='TODO: License declaration',
 tests_require=['pytest'],
 entry_points={
     'console_scripts': [
             'policy_deployment = lite6_policy_deployment_demos.policy_deployment:main'
     ],
   },
)

from generate_parameter_library_py.setup_helper import generate_parameter_module

generate_parameter_module(
  "lite6_policy_deployment_demos_parameters", # python module name for parameter library
  "config/policy.yaml", # path to input yaml file
)
