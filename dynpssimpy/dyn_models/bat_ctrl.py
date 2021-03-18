# Implement different controllers for the battery

import numpy as np


# The easiest type of control
class P_CTRL_SPEED_LOCAL:
    def __init__(self):
        self.state_list = []
        self.int_par_list = ['R'] # droop
        self.input_list = ['speed_dev']
        self.output_list = ['P_m']

    @staticmethod
    def initialize(x_0, input, output, p, int_par):
        #output['P_m'] = 1/p['R']*input['speed_dev']
        pass

    @staticmethod
    def _update(dx, x, input, output, p, int_par):
        output['P_m'] = 1/p['R']*input['speed_dev']


class WASH_LEAD_LAG:
    def __init__(self):
        self.state_list = ['x_1', 'x_2', 'x_3']
        self.int_par_list = ['delta_0']
        self.input_list = ['angle_dev']
        self.output_list = ['P_m']

    @staticmethod
    def initialize(x_0, input, output, p, int_par):
        x_0['x_1'] = 0
        x_0['x_2'] = 0
        x_0['x_3'] = 0



    @staticmethod
    def _update(dx, x, input, output, p, int_par):
        u = input['angle_dev']+int_par['delta_0']
        K = 1
        v_1 = (K*u - x['x_1']) / p['T']
        v_2 = 1 / p['T_3'] * (p['T_1'] * v_1 - x['x_2'])
        v_3 = 1 / p['T_4'] * (p['T_2'] * v_2 - x['x_3'])

        output['P_m'][:] = p['K']*v_3 # np.minimum(np.maximum(v_3, -p['H_lim']), p['H_lim'])

        dx['x_1'][:] = v_1
        dx['x_2'][:] = (p['T_1'] / p['T_3'] - 1) * v_1 - 1 / p['T_3'] * x['x_2']
        dx['x_3'][:] = (p['T_2'] / p['T_4'] - 1) * v_2 - 1 / p['T_4'] * x['x_3']

class WASH_LEAD_LAG2:
    """
    Same implementation as the VS CODE. Should be the same as the one above though
    Seems like it is able to produce the exact same results as WASH_LEAD_LAG
    """
    def __init__(self):
        self.state_list = ['x_1', 'x_2', 'x_3']
        self.int_par_list = ['delta_0']
        self.input_list = ['angle_dev']
        self.output_list = ['P_m']

    @staticmethod
    def initialize(x_0, input, output, p, int_par):
        x_0['x_1'] = 0
        x_0['x_2'] = 0
        x_0['x_3'] = 0

    @staticmethod
    def _update(dx, x, input, output, p, int_par):
        u = input['angle_dev']+int_par['delta_0']
        #u = -u
        K = 1
        v_1 = 1/p['T']*(K*u-x['x_1'])
        v_2 = p['T_1']/p['T_3']*v_1 + (1-p['T_1']/p['T_3'])*x['x_2']
        v_3 = p['T_2']/p['T_4']*v_2 + (1-p['T_2']/p['T_4'])*x['x_3']

        output['P_m'][:] = p['K']*v_3  # np.minimum(np.maximum(v_3, -p['H_lim']), p['H_lim'])

        dx['x_1'][:] = v_1
        dx['x_2'][:] = 1/p['T_3']*(v_1-x['x_2'])
        dx['x_3'][:] = 1/p['T_4']*(v_2-x['x_3'])