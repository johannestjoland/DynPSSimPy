# Trying to implement the battery model here

import numpy as np


class BAT_FIRST:
    def __init__(self):
        self.state_list = ['SOC'] # Currently only this, the other states are to be done in controller blocks
        self.input_list = ['V_t_abs', 'V_t_angle', 'p_ctrl']
        self.int_par_list = ['f', 's_n', 'V_dc',  'i_inj_q', 'i_inj_d'] # S_n is the apparent power rating in the system
        self.output_list = ['P_e', 'I_g']

    @staticmethod
    def initialize(x_0, input, output, p, int_par):
        x_0['SOC'] = 0.8 # change this if another start is wanted

        int_par['V_dc'] = 500
        int_par['i_inj_q'] = 0 # no interaction at start
        int_par['i_inj_d'] = 0

        output['P_e'] = 0
        output['I_g'] = 0

    @staticmethod
    def _current_injections(int_par):
        return int_par['i_inj_d'], int_par['i_inj_q']

    @staticmethod
    def _update(dx, x, input, output, p, int_par):

        # Converting to terminal voltage phasor (inputs are only float, not complex).
        # This is to avoid having to specify units in input_list in model definition.

        # Converting to terminal voltage phasor (inputs are only float, not complex).
        # This is to avoid having to specify units in input_list in model definition.
        v_g = input['V_t_abs']*np.exp(1j*input['V_t_angle'])  # Potential p.u. error
        angle = np.angle(input['V_t_angle']) # angle
        v_dq = v_g * np.exp(1j * (np.pi / 2 - angle))
        v_d = v_dq.real
        v_q = v_dq.imag

        # wanted power from controller
        p_m = input['p_ctrl']
        i_pu = p_m/int_par['V_dc']

        # Wanting the same current applied from all battery cells. Accounting for paralell cells when computing the power in the end
        i_cell = i_pu * (p['p_max'] / p['v_nom'])  # Not quite sure here, but it is something [kA]

        # Checking current limits
        i_cell = np.minimum(np.maximum(p['cell_imin'], i_cell), p['cell_imax'])  # [kA]

        # Finding cell voltage
        u = -i_cell / (3600 * p['cell_capacity'])

        # Checking that we dont exceed the limits
        if x['SOC'] + u < p['soc_min'] or x['SOC'] + u > p['soc_max']:
            i_cell = 0
            u = 0  # no chang in SOC in such a case, this will influence the state-variable

        # Finding internal votlage
        soc = x['SOC']
        v_int = np.minimum(
            np.maximum(p['cell_v_min'], p['cell_v_max'] * soc + p['cell_v_min'] * (1 - soc)),
            p['cell_v_max'])  # [V]
        v_cell = v_int - 1000 * i_cell * p['r_cell']  # [V]

        # Dont allow negativ DC voltage. ADDED np.minimum part newly, have to check this further
        v_cell = np.minimum(np.maximum(0, v_cell), p['cell_v_max'])  # [V]

        int_par['V_dc'] = v_cell * p['n_series'] / p['v_nom']  # [pu]
        delta_p_actual = p['eff'] * int_par['V_dc'] * i_cell * p['n_par'] * p['n_series'] / (
                    p['p_max'] / p['v_nom'])  # pu

        # Interacting BESS with the system
        delta_p_actual = delta_p_actual * p['p_max'] / (
                    int_par['s_n'] * 1000)  # getting it to the AC systems per unit apparent power

        # Update the int_par parameters
        int_par['i_inj_q'] = delta_p_actual / v_q
        int_par['i_inj_d'] = 0

        # Merging the d- and q-component of the current
        i_b_dq = int_par['i_inj_d'] + 1j * int_par['i_inj_q']

        I_g = i_b_dq * np.exp(
            -1j * (np.pi / 2 - angle))

        output['I_g'][:] = abs(I_g)
        output['P_e'][:] = int_par['i_inj_q']*v_q + int_par['i_inj_d']*v_d*int_par['s_n']

        dx['SOC'] = u