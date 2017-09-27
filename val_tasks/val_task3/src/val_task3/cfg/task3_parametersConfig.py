## *********************************************************
## 
## File autogenerated for the val_task3 package 
## by the dynamic_reconfigure package.
## Please do not edit.
## 
## ********************************************************/

from dynamic_reconfigure.encoding import extract_params

inf = float('inf')

config_description = {'upper': 'DEFAULT', 'lower': 'groups', 'srcline': 245, 'name': 'Default', 'parent': 0, 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT', 'field': 'default', 'state': True, 'parentclass': '', 'groups': [{'upper': 'STAIRWALKPOSE', 'lower': 'stairwalkpose', 'srcline': 121, 'name': 'stairWalkPose', 'parent': 0, 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT::STAIRWALKPOSE', 'field': 'DEFAULT::stairwalkpose', 'state': True, 'parentclass': 'DEFAULT', 'groups': [], 'parameters': [{'srcline': 14, 'description': 'x of goal location', 'max': 10.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/ninja/indigo_ws/src/space_robotics_challenge/val_tasks/val_task3/cfg/task3_parameters.cfg', 'name': 'x_sw', 'edit_method': '', 'default': 2.828, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 15, 'description': 'y of goal location', 'max': 1.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/ninja/indigo_ws/src/space_robotics_challenge/val_tasks/val_task3/cfg/task3_parameters.cfg', 'name': 'y_sw', 'edit_method': '', 'default': 0.292, 'level': 0, 'min': -1.0, 'type': 'double'}, {'srcline': 16, 'description': 'theta of goal location', 'max': 1.57, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/ninja/indigo_ws/src/space_robotics_challenge/val_tasks/val_task3/cfg/task3_parameters.cfg', 'name': 'theta_sw', 'edit_method': '', 'default': 0.82, 'level': 0, 'min': -1.57, 'type': 'double'}], 'type': '', 'id': 1}, {'upper': 'HANDLECENTER', 'lower': 'handlecenter', 'srcline': 121, 'name': 'handleCenter', 'parent': 0, 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT::HANDLECENTER', 'field': 'DEFAULT::handlecenter', 'state': True, 'parentclass': 'DEFAULT', 'groups': [], 'parameters': [{'srcline': 20, 'description': 'x of door handle', 'max': 10.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/ninja/indigo_ws/src/space_robotics_challenge/val_tasks/val_task3/cfg/task3_parameters.cfg', 'name': 'x_hc', 'edit_method': '', 'default': 2.828, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 21, 'description': 'y of door handle', 'max': 1.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/ninja/indigo_ws/src/space_robotics_challenge/val_tasks/val_task3/cfg/task3_parameters.cfg', 'name': 'y_hc', 'edit_method': '', 'default': 0.292, 'level': 0, 'min': -1.0, 'type': 'double'}, {'srcline': 22, 'description': 'z of door handle', 'max': 1.57, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/ninja/indigo_ws/src/space_robotics_challenge/val_tasks/val_task3/cfg/task3_parameters.cfg', 'name': 'z_hc', 'edit_method': '', 'default': 0.82, 'level': 0, 'min': -1.57, 'type': 'double'}], 'type': '', 'id': 2}, {'upper': 'TABLEWALKPOSE', 'lower': 'tablewalkpose', 'srcline': 121, 'name': 'tableWalkPose', 'parent': 0, 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT::TABLEWALKPOSE', 'field': 'DEFAULT::tablewalkpose', 'state': True, 'parentclass': 'DEFAULT', 'groups': [], 'parameters': [{'srcline': 26, 'description': 'x of goal location', 'max': 10.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/ninja/indigo_ws/src/space_robotics_challenge/val_tasks/val_task3/cfg/task3_parameters.cfg', 'name': 'x_tw', 'edit_method': '', 'default': 2.828, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 27, 'description': 'y of goal location', 'max': 1.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/ninja/indigo_ws/src/space_robotics_challenge/val_tasks/val_task3/cfg/task3_parameters.cfg', 'name': 'y_tw', 'edit_method': '', 'default': 0.292, 'level': 0, 'min': -1.0, 'type': 'double'}, {'srcline': 28, 'description': 'theta of goal location', 'max': 1.57, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/ninja/indigo_ws/src/space_robotics_challenge/val_tasks/val_task3/cfg/task3_parameters.cfg', 'name': 'theta_tw', 'edit_method': '', 'default': 0.82, 'level': 0, 'min': -1.57, 'type': 'double'}], 'type': '', 'id': 3}, {'upper': 'LEAKDETECTORLOC', 'lower': 'leakdetectorloc', 'srcline': 121, 'name': 'leakDetectorLoc', 'parent': 0, 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT::LEAKDETECTORLOC', 'field': 'DEFAULT::leakdetectorloc', 'state': True, 'parentclass': 'DEFAULT', 'groups': [], 'parameters': [{'srcline': 32, 'description': 'x of leak detector', 'max': 10.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/ninja/indigo_ws/src/space_robotics_challenge/val_tasks/val_task3/cfg/task3_parameters.cfg', 'name': 'x_ld', 'edit_method': '', 'default': 2.828, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 33, 'description': 'y of leak detector', 'max': 1.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/ninja/indigo_ws/src/space_robotics_challenge/val_tasks/val_task3/cfg/task3_parameters.cfg', 'name': 'y_ld', 'edit_method': '', 'default': 0.292, 'level': 0, 'min': -1.0, 'type': 'double'}, {'srcline': 34, 'description': 'z of leak detector', 'max': 1.57, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/ninja/indigo_ws/src/space_robotics_challenge/val_tasks/val_task3/cfg/task3_parameters.cfg', 'name': 'z_ld', 'edit_method': '', 'default': 0.82, 'level': 0, 'min': -1.57, 'type': 'double'}], 'type': '', 'id': 4}, {'upper': 'LEAKWALLPOSE', 'lower': 'leakwallpose', 'srcline': 121, 'name': 'leakWallPose', 'parent': 0, 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT::LEAKWALLPOSE', 'field': 'DEFAULT::leakwallpose', 'state': True, 'parentclass': 'DEFAULT', 'groups': [], 'parameters': [{'srcline': 38, 'description': 'x of goal location', 'max': 10.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/ninja/indigo_ws/src/space_robotics_challenge/val_tasks/val_task3/cfg/task3_parameters.cfg', 'name': 'x_lw', 'edit_method': '', 'default': 2.828, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 39, 'description': 'y of goal location', 'max': 1.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/ninja/indigo_ws/src/space_robotics_challenge/val_tasks/val_task3/cfg/task3_parameters.cfg', 'name': 'y_lw', 'edit_method': '', 'default': 0.292, 'level': 0, 'min': -1.0, 'type': 'double'}, {'srcline': 40, 'description': 'theta of goal location', 'max': 1.57, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/ninja/indigo_ws/src/space_robotics_challenge/val_tasks/val_task3/cfg/task3_parameters.cfg', 'name': 'theta_lw', 'edit_method': '', 'default': 0.82, 'level': 0, 'min': -1.57, 'type': 'double'}], 'type': '', 'id': 5}, {'upper': 'LEAKLOC', 'lower': 'leakloc', 'srcline': 121, 'name': 'leakLoc', 'parent': 0, 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT::LEAKLOC', 'field': 'DEFAULT::leakloc', 'state': True, 'parentclass': 'DEFAULT', 'groups': [], 'parameters': [{'srcline': 45, 'description': 'x of leak', 'max': 10.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/ninja/indigo_ws/src/space_robotics_challenge/val_tasks/val_task3/cfg/task3_parameters.cfg', 'name': 'x_l', 'edit_method': '', 'default': 2.828, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 46, 'description': 'y of leak', 'max': 1.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/ninja/indigo_ws/src/space_robotics_challenge/val_tasks/val_task3/cfg/task3_parameters.cfg', 'name': 'y_l', 'edit_method': '', 'default': 0.292, 'level': 0, 'min': -1.0, 'type': 'double'}, {'srcline': 47, 'description': 'z of leak', 'max': 1.57, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/ninja/indigo_ws/src/space_robotics_challenge/val_tasks/val_task3/cfg/task3_parameters.cfg', 'name': 'z_l', 'edit_method': '', 'default': 0.82, 'level': 0, 'min': -1.57, 'type': 'double'}], 'type': '', 'id': 6}, {'upper': 'REPAIRTOOLLOC', 'lower': 'repairtoolloc', 'srcline': 121, 'name': 'repairToolLoc', 'parent': 0, 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT::REPAIRTOOLLOC', 'field': 'DEFAULT::repairtoolloc', 'state': True, 'parentclass': 'DEFAULT', 'groups': [], 'parameters': [{'srcline': 51, 'description': 'x of repair tool', 'max': 10.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/ninja/indigo_ws/src/space_robotics_challenge/val_tasks/val_task3/cfg/task3_parameters.cfg', 'name': 'x_rt', 'edit_method': '', 'default': 2.828, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 52, 'description': 'y of repair tool', 'max': 1.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/ninja/indigo_ws/src/space_robotics_challenge/val_tasks/val_task3/cfg/task3_parameters.cfg', 'name': 'y_rt', 'edit_method': '', 'default': 0.292, 'level': 0, 'min': -1.0, 'type': 'double'}, {'srcline': 53, 'description': 'z of repair tool', 'max': 1.57, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/ninja/indigo_ws/src/space_robotics_challenge/val_tasks/val_task3/cfg/task3_parameters.cfg', 'name': 'z_rt', 'edit_method': '', 'default': 0.82, 'level': 0, 'min': -1.57, 'type': 'double'}], 'type': '', 'id': 7}, {'upper': 'FINISHBOXWALKPOSE', 'lower': 'finishboxwalkpose', 'srcline': 121, 'name': 'finishBoxWalkPose', 'parent': 0, 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT::FINISHBOXWALKPOSE', 'field': 'DEFAULT::finishboxwalkpose', 'state': True, 'parentclass': 'DEFAULT', 'groups': [], 'parameters': [{'srcline': 57, 'description': 'x of goal location', 'max': 10.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/ninja/indigo_ws/src/space_robotics_challenge/val_tasks/val_task3/cfg/task3_parameters.cfg', 'name': 'x_fb', 'edit_method': '', 'default': 2.828, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 58, 'description': 'y of goal location', 'max': 1.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/ninja/indigo_ws/src/space_robotics_challenge/val_tasks/val_task3/cfg/task3_parameters.cfg', 'name': 'y_fb', 'edit_method': '', 'default': 0.292, 'level': 0, 'min': -1.0, 'type': 'double'}, {'srcline': 59, 'description': 'theta of goal location', 'max': 1.57, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/home/ninja/indigo_ws/src/space_robotics_challenge/val_tasks/val_task3/cfg/task3_parameters.cfg', 'name': 'theta_fb', 'edit_method': '', 'default': 0.82, 'level': 0, 'min': -1.57, 'type': 'double'}], 'type': '', 'id': 8}], 'parameters': [], 'type': '', 'id': 0}

min = {}
max = {}
defaults = {}
level = {}
type = {}
all_level = 0

#def extract_params(config):
#    params = []
#    params.extend(config['parameters'])    
#    for group in config['groups']:
#        params.extend(extract_params(group))
#    return params

for param in extract_params(config_description):
    min[param['name']] = param['min']
    max[param['name']] = param['max']
    defaults[param['name']] = param['default']
    level[param['name']] = param['level']
    type[param['name']] = param['type']
    all_level = all_level | param['level']

