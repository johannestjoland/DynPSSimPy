import numpy as np


class SEXS:
    def __init__(self):
        self.state_list = ['x', 'e_f']
        self.int_par_list = ['x_bias']
        self.input_list = ['v_dev', 'v_pss']
        self.output_list = ['E_f']

    @staticmethod
    def initialize(x_0, input, output, p, int_par):
        bias = 1 / p['K'] * output['E_f']
        int_par['x_bias'] = bias
        x_0['x'][:] = (p['T_a'] - p['T_b']) * bias
        x_0['e_f'][:] = output['E_f']

    @staticmethod
    def _update(dx, x, input, output, p, int_par):

        u = input['v_dev'] + input['v_pss'] + int_par['x_bias']
        v_1 = 1 / p['T_b'] * (p['T_a'] * u - x['x'])

        dx['x'][:] = v_1 - u
        dx['e_f'][:] = 1/p['T_e'] * (p['K'] * v_1 - x['e_f'])

        # Lims on state variable e_f (clamping)
        lower_lim_idx = (x['e_f'] <= p['E_min']) & (dx['e_f'] < 0)
        dx['e_f'][lower_lim_idx] *= 0

        upper_lim_idx = (x['e_f'] >= p['E_max']) & (dx['e_f'] > 0)
        dx['e_f'][upper_lim_idx] *= 0

        output['E_f'][:] = np.minimum(np.maximum(x['e_f'], p['E_min']), p['E_max'])


class AVR_WITH_FEEDBACK:
    """
    Purpose: implementing control structure from assignment 7 TET4180
            without making the assumption that Ta is small enough, and avoid simplifications
    """
    def __init__(self):
        self.state_list = ['x_1', 'e_f', 'x_feedback']  # Ka = 0 in the task
        self.int_par_list = ['x_bias', 'E_f_0'] #'x_0_feedback']
        self.input_list = ['v_dev', 'v_pss', 'v_feedback']
        self.output_list = ['E_f']

    @staticmethod
    def initialize(x_0, input, output, p, int_par):
        bias = p['K_e']/ p['K_a'] * output['E_f']
        int_par['x_bias'] = bias
        x_0['x_1'][:] = bias
        x_0['e_f'][:] = output['E_f']
        int_par['E_f_0'] = output['E_f']

    @staticmethod
    def _update(dx, x, input, output, p, int_par):

        u = input['v_dev'] + input['v_pss'] + int_par['x_bias'] - input['v_feedback']
        v_1 = x['x_1']  # AFTER Ka/(1+Ta*s) block, when Ka = 0
        v_2 = x['e_f']
        v_3 = 1/p['T_f']*(p['K_f']*(int_par['E_f_0']-v_2)-x['x_feedback'])
        input['v_feedback'] = v_3

        dx['x_1'] = 1/p['T_a']*(p['K_a']*u-x['x_1'])
        dx['e_f'][:] = 1/(p['T_e']/p['K_e'])*(1/p['K_e']*v_1-x['e_f'])
        dx['x_feedback'] = v_3

        # Lims on state variable e_f (clamping)
        lower_lim_idx = (x['e_f'] <= p['E_min']) & (dx['e_f'] < 0)
        dx['e_f'][lower_lim_idx] *= 0

        upper_lim_idx = (x['e_f'] >= p['E_max']) & (dx['e_f'] > 0)
        dx['e_f'][upper_lim_idx] *= 0

        output['E_f'][:] = np.minimum(np.maximum(x['e_f'], p['E_min']), p['E_max'])


class MYAVR:
    def __init__(self):
        self.state_list = ['xa', 'e_f']
        self.int_par_list = []
        self.input_list = ['v_dev', 'v_pss']
        self.output_list = ['E_f']

    @staticmethod
    def initialize(x_0, input, output, p, int_par):
        x_0['x'][:] = (p['T_a'] - p['T_b'])
        x_0['e_f'][:] = output['E_f']

    @staticmethod
    def _update(dx, x, input, output, p, int_par):

        u = input['v_dev'] + input['v_pss'] + int_par['x_bias']
        v_1 = 1 / p['T_b'] * (p['T_a'] * u - x['x'])

        dx['x'][:] = v_1 - u
        dx['e_f'][:] = 1/p['T_e'] * (p['K'] * v_1 - x['e_f'])

        # Lims on state variable e_f (clamping)
        lower_lim_idx = (x['e_f'] <= p['E_min']) & (dx['e_f'] < 0)
        dx['e_f'][lower_lim_idx] *= 0

        upper_lim_idx = (x['e_f'] >= p['E_max']) & (dx['e_f'] > 0)
        dx['e_f'][upper_lim_idx] *= 0

        output['E_f'][:] = np.minimum(np.maximum(x['e_f'], p['E_min']), p['E_max'])



if __name__ == '__main__':
    # Simple speed test of model (jit vs nojit)
    import time
    from numba import jit


    n_units = 2000

    import dynpssimpy.dynamic as dps
    import ps_models.k2a as model_data
    model = model_data.load()
    ps = dps.PowerSystemModel(model)
    ps.power_flow()
    ps.init_dyn_sim()

    mdl = SEXS()
    n_states = len(mdl.state_list)
    state_desc_mdl = np.vstack([np.repeat(['']*n_units, n_states), np.tile(mdl.state_list, n_units)]).T
    mdl.par = np.concatenate([ps.avr_mdls['SEXS'].par[0:1]] * n_units)
    mdl.state_idx = np.zeros((n_units,), dtype=[(state, int) for state in mdl.state_list])
    for i, state in enumerate(mdl.state_list):
        mdl.state_idx[state] = np.where(state_desc_mdl[:, 1] == state)[0]

    mdl.int_par = np.array(np.zeros(n_units), [(par, float) for par in mdl.int_par_list])

    mdl.idx = slice(0, n_units * n_states)
    mdl.dtypes = [(state, np.float) for state in mdl.state_list]

    # x = np.zeros(2*n)
    dm = mdl
    x = np.arange(2 * n_units, dtype=float)
    dx = np.arange(2 * n_units, dtype=float)
    # update_jit = jit()(avr._update)
    # update_jit(dx, x, 1, avr.par, avr.state_idx, avr.int_par)

    n_it = 1000
    t_0 = time.time()
    for _ in range(n_it):
        mdl._update(
            dx[dm.idx].view(dtype=dm.dtypes),
            x[dm.idx].view(dtype=dm.dtypes),
            1, mdl.par, mdl.int_par)
    print(time.time() - t_0)

    from numba import jit
    update_jit = jit()(mdl._update)
    update_jit(
        dx[dm.idx].view(dtype=dm.dtypes),
        x[dm.idx].view(dtype=dm.dtypes),
        1, mdl.par, mdl.int_par)

    t_0 = time.time()
    for _ in range(n_it):
        update_jit(
            dx[dm.idx].view(dtype=dm.dtypes),
            x[dm.idx].view(dtype=dm.dtypes),
            1, mdl.par, mdl.int_par)
    print(time.time() - t_0)

    # t_0 = time.time()
    # for _ in range(n_it):
    #     update_jit(dx, x, 1, avr.par, avr.state_idx, avr.int_par)
    # print(time.time() - t_0)