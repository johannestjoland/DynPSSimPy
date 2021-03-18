import dynpssimpy.dynamic as dps
import dynpssimpy.modal_analysis as dps_mdl
import dynpssimpy.plotting as dps_plt
import ps_models.k2a as model_data
import importlib
import numpy as np
import matplotlib.pyplot as plt

if __name__ == '__main__':
    import ps_models.n44 as model_data
    ps = dps.PowerSystemModel(model_data.load())
    ps.power_flow()
    ps.init_dyn_sim()

    # Perform system linearization
    ps_lin = dps_mdl.PowerSystemModelLinearization(ps)
    ps_lin.linearize()

    # Alternatively:
    # ps_lin = ps.linearize()

    # Plot eigenvalues
    dps_plt.plot_eigs(ps_lin.eigs)
    print(ps_lin.eigs)
    # Get mode shape for electromechanical modes
    mode_idx = ps_lin.get_mode_idx(['em'], damp_threshold=0.3)
    rev = ps_lin.rev
    mode_shape = rev[np.ix_(ps.gen_mdls['GEN'].state_idx['speed'], mode_idx)]

    """
    fig33, ax33 = plt.subplots(1)
    (-0.276, 4.05)
    (-0.431, 5.761)
    time = np.arange(0, 50, 0.0001);
    y1 = 3*np.exp(-0.36/4*time)*np.sin(2.44*time)
    y2 = np.exp(-0.276/4*time)*np.sin(4.05*time)
    y3 = np.exp(-0.431/4*time)*np.sin(-5.761*time)
    y4 = np.exp(-0.519/4*time)*np.sin(6.583*time)
    y5 = np.exp(-0.721/4*time)*np.sin(6.71*time)
    #ax33.plot(time, y1, color='black')
    #ax33.plot(time, y2, color='blue')
    #ax33.plot(time, y3, color='r')
    ax33.plot(time, y3+y4, color='g')
    ax33.grid(True)
    """

    """
    # Plot mode shape
    if len(mode_idx) > 0:
        fig, ax = plt.subplots(1, mode_shape.shape[1], subplot_kw={'projection': 'polar'})
        for ax_, ms in zip(ax, mode_shape.T):
            dps_plt.plot_mode_shape(ms, ax=ax_, normalize=True)
    """
    plt.show()