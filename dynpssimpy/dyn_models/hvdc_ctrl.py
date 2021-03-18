# Implementing control for obtaining additional p-input-signal

import numpy as np

class HVDC_PROP_CTRL:
    def __init__(self):
        self.state_list = [] # filtered derivative
        self.input_list = ['V_t_angle_1', 'V_t_angle_2']
        self.int_par_list = ['s_n', 'delta_v_angles']
        self.output_list = ['p_ctrl']

    @staticmethod
    def initialize(x_0, input, output, p, int_par):
        output['p_ctrl'] = 0
        int_par['delta_v_angles'] = input['V_t_angle_1'] - input['V_t_angle_2']

    @staticmethod
    def _update(dx, x, input, output, p, int_par):
        # Make sure the relative difference between the signals are the same!
        if input['V_t_angle_1']-input['V_t_angle_2'] < -np.pi:
            input['V_t_angle_1'] += 2*np.pi
        elif input['V_t_angle_2']-input['V_t_angle_1'] < -np.pi:
            input['V_t_angle_2'] += 2 * np.pi

        u = (input['V_t_angle_1'] -  input['V_t_angle_2']) - int_par['delta_v_angles']
        output['p_ctrl'][:] = p['K']*u  # np.minimum(np.maximum(v_3, -p['H_lim']), p['H_lim'])

class HVDC_PD_CTRL:
    def __init__(self):
        self.state_list = ['x_1'] # filtered derivative
        self.input_list = ['V_t_angle_1', 'V_t_angle_2']
        self.int_par_list = ['s_n', 'delta_v_angles']
        self.output_list = ['p_ctrl']

    @staticmethod
    def initialize(x_0, input, output, p, int_par):
        x_0['x_1'] = 0
        output['p_ctrl'] = 0
        int_par['delta_v_angles'] = input['V_t_angle_1'] - input['V_t_angle_2']

    @staticmethod
    def _update(dx, x, input, output, p, int_par):
        u = (input['V_t_angle_1'] -  input['V_t_angle_2']) - int_par['delta_v_angles']

        T = 0.1
        v_1 = 1/T*(u-x['x_1'])

        output['p_ctrl'][:] = p['K']*u + v_1 # np.minimum(np.maximum(v_3, -p['H_lim']), p['H_lim'])
        dx['x_1'] = v_1

class HVDC_WASH_LEAD_LAG:
    def __init__(self):
        self.state_list = ['x_1', 'x_2', 'x_3',] # filtered derivative
        self.input_list = ['V_t_angle_1', 'V_t_angle_2']
        self.int_par_list = ['s_n', 'delta_v_angles']
        self.output_list = ['p_ctrl']

    @staticmethod
    def initialize(x_0, input, output, p, int_par):
        x_0['x_1'] = 0
        x_0['x_2'] = 0
        x_0['x_3'] = 0
        output['p_ctrl'] = 0
        int_par['delta_v_angles'] = input['V_t_angle_1'] - input['V_t_angle_2']

    @staticmethod
    def _update(dx, x, input, output, p, int_par):

        # Make sure the relative difference between the signals are the same!
        if input['V_t_angle_1']-input['V_t_angle_2'] < -np.pi:
            input['V_t_angle_1'] += 2*np.pi
        elif input['V_t_angle_2']-input['V_t_angle_1'] < -np.pi:
            input['V_t_angle_2'] += 2 * np.pi

        u = input['V_t_angle_1']-input['V_t_angle_2'] - int_par['delta_v_angles']

        K = 1
        T = 10
        T1 = 0.96
        T3 = 0.165

        v_1 = 1/T * (K*u - x['x_1'])
        v_2 = T1/T3 * v_1 + (1 - T1/T3) * x['x_2']
        v_3 = T1/T3 * v_2 + (1 - T1/T3) * x['x_3']

        output['p_ctrl'][:] = p['K']*v_3
        #output['p_ctrl'][:] =  np.minimum(np.maximum(output['p_ctrl'][:], -0.1, 0.1))

        #print('u: ', u)
        #print('v_1: ', v_1)
        #print('v_2: ', v_2)
        #print('v_3: ', v_3)

        dx['x_1'][:] = v_1
        dx['x_2'][:] = 1/T3*(v_1 - x['x_2'])
        dx['x_3'][:] = 1/T3*(v_2 - x['x_3'])

"""
class HVDC_WASH_LEAD_LAG:
    def __init__(self):
        self.state_list = ['x_1', 'x_2', 'x_3']
        self.int_par_list = ['delta_0']
        self.input_list = ['angle_dev']
        self.output_list = ['P_m']

    @staticmethod
    def initialize(x_0, input, output, p, int_par):
        pass


    @staticmethod
    def _update(dx, x, input, output, p, int_par):
        u = input['angle_dev']+int_par['delta_0']

        v_1 = (p['K'] * u - x['x_1']) / p['T']
        v_2 = 1 / p['T_3'] * (p['T_1'] * v_1 - x['x_2'])
        v_3 = 1 / p['T_4'] * (p['T_2'] * v_2 - x['x_3'])

        output['P_m'][:] = v_3 # np.minimum(np.maximum(v_3, -p['H_lim']), p['H_lim'])

        dx['x_1'][:] = v_1
        dx['x_2'][:] = (p['T_1'] / p['T_3'] - 1) * v_1 - 1 / p['T_3'] * x['x_2']
        dx['x_3'][:] = (p['T_2'] / p['T_4'] - 1) * v_2 - 1 / p['T_4'] * x['x_3']
"""



