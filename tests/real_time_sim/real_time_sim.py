import dynpssimpy.dynamic as dps
from collections import defaultdict
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import RK23, RK45, solve_ivp
import importlib
import time


if __name__ == '__main__':
    importlib.reload(dps)

    # Load model
    import ps_models.ieee39 as model_data
    # import ps_models.ieee39 as model_data
    # import ps_models.sm_ib as model_data
    # import ps_models.sm_load as model_data
    model = model_data.load()

    # model['avr'] = dict()
    # model['gov'] = dict()
    # model['pss'] = dict()
    t_0 = time.time()

    ps = dps.PowerSystemModel(model=model)
    ps.pf_max_it = 100
    ps.power_flow()
    ps.init_dyn_sim()

    t_end = 10
    x0 = ps.x0.copy()
    # x0[ps.angle_idx[0]] += 1
    print(np.max(abs(ps.ode_fun(0, ps.x0))))

    sol = RK23(ps.ode_fun, 0, x0, t_end, max_step=20e-3)

    t = 0
    result_dict = defaultdict(list)

    t_start_sim = time.time()

    while t < t_end:

        # Simulate next step
        result = sol.step()
        x = sol.y
        t = sol.t

        t_world = time.time() - t_start_sim
        t_err = sol.t - t_world
        print(t, t_err)
        # if t_err > 0:
        #     time.sleep(t_err)

        ps.y_bus_red_mod[np.diag_indices(ps.n_bus_red)] += np.random.rand(ps.n_bus_red)*0.001

        # Store result
        result_dict['Global', 't'].append(sol.t)
        [result_dict[tuple(desc)].append(state) for desc, state in zip(ps.state_desc, x)]

    index = pd.MultiIndex.from_tuples(result_dict)
    result = pd.DataFrame(result_dict, columns=index)

    fig, ax = plt.subplots(2)
    ax[0].plot(result[('Global', 't')], result.xs(key='speed', axis='columns', level=1))
    ax[1].plot(result[('Global', 't')], result.xs(key='angle', axis='columns', level=1))
    plt.show(True)
