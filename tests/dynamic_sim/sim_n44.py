# For running simulation in N44


import dynpssimpy.dynamic as dps
from collections import defaultdict
import pandas as pd
import matplotlib.pyplot as plt
import dynpssimpy.utility_functions as dps_uf
import importlib
import sys
import time
import numpy as np
import dynpssimpy.modal_analysis as dps_mdl
import dynpssimpy.plotting as dps_plt
# Line outage when HYGOV is used
# Added for not making fatal changes to the original file


if __name__ == '__main__':
    importlib.reload(dps)

    # Load model
    import ps_models.n44 as model_data
    # import ps_models.ieee39 as model_data
    # import ps_models.sm_ib as model_data
    # import ps_models.sm_load as model_data
    model = model_data.load()

    """
    # Decreasing generator and load power
    for idx, gen in enumerate(model['generators']['GEN']):
        if idx == 0: continue
        gen[4] = gen[4]/2

    for idx, load in enumerate(model['loads']):
        if idx == 0: continue
        load[2] = load[2]/2  # decreasing active power
        load[3] = load[3]/2  # decreasing reactive power """
    for idx, avr in enumerate(model['avr']['SEXS']):
        if idx == 0: continue
        avr[2] = avr[2]/1.5
        avr[6] = -10
        avr[7] = 10

    # Calling model twice, first is to get the desired names and so on.
    # Could probably do it another way, but this works
    ps = dps.PowerSystemModel(model=model)

    # Add controls for all generators (not specified in model)

    """
    model['gov'] = {'TGOV1':
                        [['name', 'gen', 'R', 'D_t', 'V_min', 'V_max', 'T_1', 'T_2', 'T_3']] +
                        [['GOV' + str(i), gen_name, 0.05, 0, 0, 1, 0.2, 1, 2] for i, (gen_name, gen_p) in
                         enumerate(zip(ps.generators['name'], ps.generators['P']))]
                    }
    #model.pop('gov')
    model['avr'] = {'SEXS':
                        [['name', 'gen', 'K', 'T_a', 'T_b', 'T_e', 'E_min', 'E_max']] +
                        [['AVR' + str(i), gen_name, 100, 2.0, 10.0, 0.5, -3, 3] for i, gen_name in
                         enumerate(ps.generators['name'])]
                    }
    #model.pop('avr')
    model['pss'] = {'STAB1':
                        [['name', 'gen', 'K', 'T', 'T_1', 'T_2', 'T_3', 'T_4', 'H_lim']] +
                       [['PSS' + str(i), gen_name, 50, 10.0, 0.5, 0.5, 0.05, 0.05, 0.03] for i, gen_name in
                         enumerate(ps.generators['name'])]
                    }
    model.pop('pss')
    """

    # Power system with governos, avr and pss
    ps = dps.PowerSystemModel(model=model)
    ps.use_numba = True


    ps.pf_max_it = 10
    ps.power_flow()
    ps.init_dyn_sim()

    # Solver
    t_end = 180
    sol = dps_uf.ModifiedEuler(ps.ode_fun, 0, ps.x0, t_end, max_step=30e-3)

    t = 0
    result_dict = defaultdict(list)
    t_0 = time.time()

    gen_vars = ['P_e', 'I_g','P_m']
    load_vars = ['P_l']  # l subscript means "load"
    hvdc_vars = ['P_e_1', 'P_e_2', 'I', 'i_inj_q_1_out', 'i_inj_q_2_out', 'p_ctrl', 'V_t_angle_1', 'V_t_angle_2', 'ang_dq1', 'ang_dq2']

    gen_var_desc = ps.var_desc('GEN',gen_vars)
    load_var_desc = ps.var_desc('load',load_vars)
    hvdc_var_desc = ps.var_desc('HVDC', hvdc_vars)

    event_flag = True
    event_flag2 = True
    event_flag_mode = True
    event_flag_mode2 = True
    event_flag_mode3 = True
    event_flag_mode4 = True

    fig_mode, ax_mode = plt.subplots(1)
    # Perform system linearization
    ps_lin = dps_mdl.PowerSystemModelLinearization(ps)
    ps_lin.linearize()
    dps_plt.plot_eigs_2(ps_lin.eigs, ax_mode, fig_mode, col=[1, 0, 0], label='t={}'.format(t))

    number_of_mode_plots = 37
    mode_flags = [True]*number_of_mode_plots # 25 seconds, first time this oscillation occurs
    mode_times = np.arange(0, number_of_mode_plots, 1)
    current_mode_time_idx = 0
    fig_cycle, ax_cycle = plt.subplots(1)
    while t < t_end:
        sys.stdout.write("\r%d%%" % (t/(t_end)*100))

        # Simulate next step
        result = sol.step()
        x = sol.y
        t = sol.t


        if t > 2 and event_flag:
            event_flag = False
            #ps.network_event('sc', '3359', 'connect')
            #ps.network_event('load_increase', 'B9', 'connect')
            #ps.network_event('line', 'L3359-5101-1', 'disconnect')
            ps.network_event('line', 'L3100-3200-1', 'disconnect')
            # Load change doesnt care about connect or disconnect, the sign on the value (MW) is whats interesting
            #ps.network_event('load_change', 'L3359-1', 'connect', value=100)
        if t >  t_end + 2.02 and event_flag2:
            event_flag2 = False
            ps_lin = dps_mdl.PowerSystemModelLinearization(ps)
            ps_lin.linearize(x0=x)
            dps_plt.plot_eigs(ps_lin.eigs)
            #ps.network_event('load_change', 'L3359-1', 'connect', value=-1000)
            ps.network_event('line', 'L3100-3200-1', 'connect')
            #ps.network_event('sc', '3359', 'disconnect')

        if t > 25 and event_flag_mode:
            event_flag_mode = False
            ps_lin = dps_mdl.PowerSystemModelLinearization(ps)
            ps_lin.linearize(x0=x)
            dps_plt.plot_eigs_2(ps_lin.eigs, ax_mode, fig_mode, col=[0, 0, 0.7], label='t={}'.format(t))

        if t > 44 and event_flag_mode2:
            event_flag_mode2 = False
            ps_lin = dps_mdl.PowerSystemModelLinearization(ps)
            ps_lin.linearize(x0=x)
            dps_plt.plot_eigs_2(ps_lin.eigs, ax_mode, fig_mode, col=[0, 0, 1.0], label='t={}'.format(t))
        if t > 65 and event_flag_mode3:
            event_flag_mode3 = False
            ps_lin = dps_mdl.PowerSystemModelLinearization(ps)
            ps_lin.linearize(x0=x)
            dps_plt.plot_eigs_2(ps_lin.eigs, ax_mode, fig_mode, col=[0,0.7,0], label='t={}'.format(t))
        if t > t_end - 0.04 and event_flag_mode4:
            event_flag_mode4 = False
            ps_lin = dps_mdl.PowerSystemModelLinearization(ps)
            ps_lin.linearize(x0=x)
            dps_plt.plot_eigs_2(ps_lin.eigs, ax_mode, fig_mode, col=[0,0.0,0], label='t={}'.format(t))

        if True: # Plotting the eigenvalues towards the end of simulation for each second
            if current_mode_time_idx < len(mode_flags):
                if t > mode_times[current_mode_time_idx]+t_end - 50 and mode_flags[current_mode_time_idx]:
                    mode_flags[current_mode_time_idx] = False
                    ps_lin = dps_mdl.PowerSystemModelLinearization(ps)
                    ps_lin.linearize(x0=x)
                    dps_plt.plot_eigs_2(ps_lin.eigs, ax_cycle, fig_cycle, col=[0,0, current_mode_time_idx/number_of_mode_plots], label='t={:.1f}'.format(t))
                    current_mode_time_idx += 1

        #if t > 3.28: exit()
        # Store result
        result_dict['Global', 't'].append(sol.t)
        [result_dict[tuple(desc)].append(state) for desc, state in zip(ps.state_desc, x)]

        # Store generator values
        ps.store_vars('GEN', gen_vars, gen_var_desc, result_dict)
        ps.store_vars('HVDC', hvdc_vars, hvdc_var_desc, result_dict)

        # Store ACE signals
        if bool(ps.ace_mdls):
            for key, dm in ps.ace_mdls.items():
                # Store ACE signals
                [result_dict[tuple([n, 'ace'])].append(ace) for n, ace in zip(dm.par['name'], dm.ace)]
                # Store instantaneous line flows
                [result_dict[tuple([n, 'p_tie'])].append(p_tie*ps.s_n) for n, p_tie in zip(dm.par['name'], dm.p_tie)]
                # Store scheduled P_tie values
                [result_dict[tuple([n, 'p_tie0'])].append(p_tie0 * ps.s_n) for n, p_tie0 in zip(dm.par['name'], dm.int_par['Ptie0'])]

    print('\nSimulation completed in {:.2f} seconds.'.format(time.time() - t_0))

    ax_mode.legend()
    ax_cycle.legend()
    index = pd.MultiIndex.from_tuples(result_dict)
    result = pd.DataFrame(result_dict, columns=index)

    fig, ax = plt.subplots(2)
    ax[0].plot(result[('Global', 't')], result.xs(key='speed', axis='columns', level=1))
    ax[0].set_title('Speed')
    ax[0].grid(True)
    ax[1].plot(result[('Global', 't')], result.xs(key='angle', axis='columns', level=1))

    # Plot ACEs if these are included
    if bool(ps.ace_mdls):
        fig2, ax2 = plt.subplots(2)
        ax2[0].plot(result[('Global', 't')], result.xs(key='ace', axis='columns', level=1))
        ax2[0].set_title('ACE (top) and P-tie (bottom)')
        ax2[0].set_ylabel('ACE')
        ax2[1].plot(result[('Global', 't')], result.xs(key='p_tie', axis='columns', level=1))
        ax2[1].set_ylabel('P [MW]')
        # Append plot of scheduled value of active power transfer
        ax2[1].plot(result[('Global', 't')], result.xs(key='p_tie0', axis='columns', level=1))
        ax2[0].grid(True)
        ax2[1].grid(True)

    # Plot Pe to see how much each generator outputs
    var3 = 'P_e'  # variable to plot
    p3 = result.xs(key=var3, axis='columns', level=1)
    legnd3 = list(np.array(var3 + ': ') + p3.columns)
    fig3, ax3 = plt.subplots(1)
    ax3.plot(result[('Global', 't')], p3)
    ax3.set_title('Pe')
    ax3.set_ylabel('Pe [MW]')
    ax3.set_xlabel('Time [s]')
    ax3.legend(legnd3)
    ax3.grid(True)

    fig4, ax4 = plt.subplots(5)
    p_e_1 = result.xs(key='P_e_1', axis='columns', level=1)
    p_e_2 = result.xs(key='P_e_2', axis='columns', level=1)
    legnd4 = list(np.array('P_e_1' + ': ') + p_e_1.columns)
    legnd5 = list(np.array('P_e_2' + ': ') + p_e_2.columns)
    ax4[0].plot(result[('Global', 't')], p_e_1)
    ax4[0].plot(result[('Global', 't')], p_e_2)
    ax4[0].grid(True)
    ax4[0].legend(legnd4+legnd5)

    ax4[1].plot(result[('Global', 't')], result.xs(key='p_ctrl', axis='columns', level=1))
    ax4[2].plot(result[('Global', 't')], result.xs(key='V_t_angle_1', axis='columns', level=1))
    ax4[2].plot(result[('Global', 't')], result.xs(key='V_t_angle_2', axis='columns', level=1))
    ax4[2].plot(result[('Global', 't')], result.xs(key='V_t_angle_1', axis='columns', level=1)-result.xs(key='V_t_angle_2', axis='columns', level=1))
    ax4[1].legend(['p_ctrl'])
    ax4[2].legend(['V_t_angle_1', 'V_t_angle_2', 'delta_v_angle'])

    ax4[3].plot(result[('Global', 't')], result.xs(key='i_inj_q_1_out', axis='columns', level=1))
    ax4[3].plot(result[('Global', 't')], result.xs(key='i_inj_q_2_out', axis='columns', level=1))
    ax4[3].legend(['i_inj_q_1', 'i_inj_q_2'])

    ax4[4].plot(result[('Global', 't')], result.xs(key='ang_dq1', axis='columns', level=1))
    ax4[4].plot(result[('Global', 't')], result.xs(key='ang_dq2', axis='columns', level=1))
    ax4[4].legend(['ang_dq1', 'ang_dq2'])


    plt.show()