# Implementing a HVDC link

import numpy as np


class HVDC_SIMPLE:
    def __init__(self):
        self.state_list = ['i_inj_q_1'] # 1 is for the first bus, two is for the second bus
        self.input_list = ['V_t_abs_1', 'V_t_abs_2', 'V_t_angle_1', 'V_t_angle_2', 'p_ctrl', 'ang_dq1', 'ang_dq2'] # ang_dq1 for doing the transformation
        self.int_par_list = ['f', 's_n', 'p_ref', 'i_inj_q_1_ref', 'i_inj_q_2_ref', 'i_inj_q_2'] # p_ref is the scheduled HVDC power in pu
        self.output_list = ['P_e_1', 'P_e_2', 'I', 'i_inj_q_1_out', 'i_inj_q_2_out']

    @staticmethod
    def initialize(x_0, input, output, p, int_par):
        v_g_1 = input['V_t_abs_1']*np.exp(1j*input['V_t_angle_1'])  # Potential p.u. error
        #angle_1 = np.angle(v_g_1)
        angle_1 = input['ang_dq1']
        v_dq_1 = v_g_1 * np.exp(1j * (np.pi / 2 - angle_1))
        v_q_1 = v_dq_1.imag

        v_g_2 = input['V_t_abs_2']*np.exp(1j*input['V_t_angle_2'])  # Potential p.u. error
        #angle_2 = np.angle(v_g_2)
        angle_2 = input['ang_dq2']
        v_dq_2 = v_g_2 * np.exp(1j * (np.pi / 2 - angle_2))
        v_q_2 = v_dq_2.imag

        # Set to zero, only looking at deviations from the scheduled value
        x_0['i_inj_q_1'][:] = 0
        int_par['i_inj_q_2'] = 0

        output['P_e_1'] = int_par['p_ref']
        int_par['i_inj_q_1_ref'] = output['P_e_1']/v_q_1
        output['I'] = output['P_e_1']/input['V_t_abs_1']

        output['P_e_2'] = -(output['P_e_1']+np.abs(p['R']*output['I']))
        # Could have this as state, but it is dependent on the i_inj_q_1 current, so easier this way
        int_par['i_inj_q_2_ref'] = output['P_e_2']/v_q_2

        output['i_inj_q_1_out'] = x_0['i_inj_q_1']
        output['i_inj_q_2_out'] = int_par['i_inj_q_2']

    @staticmethod
    def _current_injections(x, int_par):
        return x['i_inj_q_1']+int_par['i_inj_q_1_ref'], int_par['i_inj_q_2']+int_par['i_inj_q_2_ref']
        #return int_par['i_inj_q_1'] + int_par['i_inj_q_1_ref'], int_par['i_inj_q_2'] + int_par['i_inj_q_2_ref']

    @staticmethod
    def _update(dx, x, input, output, p, int_par):
        # update the state variables here depending on the control signal input
        v_g_1 = input['V_t_abs_1']*np.exp(1j*input['V_t_angle_1'])  # Potential p.u. error
        #angle_1 = np.angle(v_g_1)
        angle_1 = input['ang_dq1']
        v_dq_1 = v_g_1 * np.exp(1j * (np.pi / 2 - angle_1))
        v_q_1 = v_dq_1.imag

        v_g_2 = input['V_t_abs_2']*np.exp(1j*input['V_t_angle_2'])  # Potential p.u. error
        #angle_2 = np.angle(v_g_2)
        angle_2 = input['ang_dq2']
        v_dq_2 = v_g_2 * np.exp(1j * (np.pi / 2 - angle_2))
        v_q_2 = v_dq_2.imag

        # Possibly a minus-sign here
        u = input['p_ctrl']/v_q_1

        output['P_e_1'] = v_q_1 * x['i_inj_q_1'] + int_par['p_ref']
        output['I'] = output['P_e_1']/input['V_t_abs_1']

        # Positive P_e_1 -> power flowing to this bus -> higher power magnitude at bus 2
        # If opposite, want lower absolute value of power at bus 2
        output['P_e_2'] = -(output['P_e_1'] + np.abs(p['R'] * output['I']))

        # Same sign convetion for these
        delta_i = v_q_1*x['i_inj_q_1']/input['V_t_abs_1']
        int_par['i_inj_q_2'] = -(v_q_1*x['i_inj_q_1'] + np.abs(p['R'] * delta_i))/v_q_2


        output['i_inj_q_1_out'] = x['i_inj_q_1'] # x['i_inj_q_1']
        output['i_inj_q_2_out'] = int_par['i_inj_q_2']

        dx['i_inj_q_1'] = 1/p['T']*(p['K']*u-x['i_inj_q_1'])
