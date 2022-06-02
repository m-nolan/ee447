import itertools
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from tqdm import tqdm
import control

SETTLE_THRESH = 1.02
TS_REF = 1.33
PO_REF = 1.01
SSE_REF = 0.0
CU_REF = 1.5e3
GM_REF = 20.0
T_MAX = 6
T_NUM = 100
NVALS = 10

# optimization schemes, i.e. what to optimize
W_SCHEME = np.array(
    [
        [1.0, 0.0, 0.0, 0.0, 0.0],  # settling time
        [0.0, 1.0, 0.0, 0.0, 0.0],  # overshoot
        [0.0, 0.0, 1.0, 0.0, 0.0],  # SSE
        [0.0, 0.0, 0.0, 1.0, 0.0],  # control effort
        [0.0, 0.0, 0.0, 0.0, 1.0],  # gain margin
        [0.2, 0.2, 0.2, 0.2, 0.2],  # equal balance
    ]
)

# control system creation

def ctrl_PID(kp, ki, kd, pp):
    """ctrl_PID

    Args:
        kp (float): proportional gain for PID controller
        ki (float): integrator gain for PID controller
        kd (float): derivative gain for PID controller
        pp (float): offset for PID controller

    Returns:
        ctrl: control system block model specified by PID input
    """
    return pp * control.tf([kd, kp, ki], [1, pp, 0])

# control system simulation
def sim_PID(plant, ctrl, t_max=T_MAX, t_num=T_NUM):
    """sim_PID

    Args:
        plant (TransferFunction): plant model transfer function
        ctrl (TransferFunction): controller model transfer function
        t_max (float, optional): simulation length. Defaults to T_MAX.
        t_num (int, optional): number of simulation time steps. Defaults to T_NUM.

    Returns:
        sim: system impulse response simulation
        ctrl_sim: control system impulse response simulation
    """
    ctrl_u = ctrl / (1 + ctrl*plant) # H feedback assumed 1
    sys = plant * ctrl_u
    sim = control.step_response(sys, t_max, 0, T_num=t_num)
    ctrl_sim = control.step_response(ctrl_u, t_max, 0, T_num=t_num)
    
    return sim, ctrl_sim

# control system performance measures

def settling_time(sim):
    """settling_time

    Args:
        sim (control.timeresp.TimeResponseData): simulation of a system step response

    Returns:
        settling_time (float): system step response settling time. If the step response does not reach the settling threshold within the simulation time, return np.inf.
    """
    
    time = sim.t.squeeze()
    y = sim.y.squeeze()
    y_final = y[-1]
    upper_bound = y_final * SETTLE_THRESH
    lower_bound = y_final / SETTLE_THRESH
    try:
        settle_idx = [idx for idx in range(len(y)) 
            if np.logical_and(y[idx:] > lower_bound, y[idx:] < upper_bound).all()][0]
        settle_time = time[settle_idx]
    except:
        settle_time = np.inf
    return settle_time

def overshoot_ratio(sim):
    """overshoot_ratio

    Args:
        sim (control.timeresp.TimeResponseData): simulation of a system step response

    Returns:
        overshoot (float): ratio of a system's maximum value and final value
    """

    y = sim.y.squeeze()
    y_max = np.max(y)
    y_final = y[-1]
    return y_max / y_final

def steady_state_error(sim):
    """steady_state_error

    Args:
        sim (control.timeresp.TimeResponseData): simulation of a system step response

    Returns:
        sse (float): steady state error, estiamted from end of step response simulation
    """

    y = sim.y.squeeze()
    y_final = y[-1]
    return np.abs(1.0-y_final)

def control_effort(ctrl_sim):
    """control_effort

    Args:
        ctrl_sim (control.timeresp.TimeResponseData): simulation of a control block step reponse

    Returns:
        ctrl_eff (float): maximal output of the control block
    """
    
    u = ctrl_sim.y.squeeze()
    return np.max(np.abs(u))

gm_max = 1e4
def gain_margin(sys):
    """gain_margin

    Args:
        sys (TransferFunction): product of system plant and controller model transfer functions

    Returns:
        gm (float): computes gain margin from total system transfer function
    """
    gm, pm, wcg, wcp = control.margin(sys)
    if gm > gm_max:
        gm = gm_max
    return gm

# optimization
def cost_PID(plant, ctrl):
    """cost_PID

    computes PID controller cost metrics

    Args:
        plant (TransferFunction): plant model transfer function
        ctrl (TransferFunction): controller model transfer function

    Returns:
        ts (float): system settling time
        po (float): system percent overshoot (ratio of max output to 1)
        sse (float): steady state error
        cu (float): control effort (maximum controller output)
        gm (float): gain margin
        y (np.array): plant step response
    """
    sim, ctrl_sim = sim_PID(plant, ctrl)
    ts = settling_time(sim)
    po = overshoot_ratio(sim)
    sse = steady_state_error(sim)
    cu = control_effort(ctrl_sim)
    gm = gain_margin(ctrl*plant)
    y = sim.y.squeeze()

    return ts, po, sse, cu, gm, y

def err_PID(ts, po, sse, cu, gm, ref_dict = {}):
    """err_PID

    Args:
        ts (float): system settling time
        po (float): system percent overshoot (ratio of max output to 1)
        sse (float): steady state error
        cu (float): control effort (maximum controller output)
        gm (float): gain margin
        ref_dict (dict, optional): reference value dictionary. Defaults to {}.

    Returns:
        err (np.array): array of error values
    """
    ts_ref = ref_dict.get('ts',TS_REF)
    po_ref = ref_dict.get('po',PO_REF)
    cu_ref = ref_dict.get('cu',CU_REF)
    gm_ref = ref_dict.get('gm',GM_REF)
    err = np.array([
        np.abs(ts-ts_ref)/ts_ref,
        np.abs(po-po_ref)/po_ref,
        sse,
        cu/cu_ref,
        np.abs(gm-gm_ref)/gm_ref
    ])
    return err

def grid_search_PID(plant,kp_lim,ki_lim,kd_lim,pp,w_scheme,nvals):
    """grid_search_PID

    Args:
        plant (TransferFunction): system plant model transfer function
        kp_lim (tuple): (min, max) proportional gain for PID controller
        ki_lim (tuple): (min, max) integrative gain for PID controller
        kd_lim (tuple): (min, max) derivative gain for PID controller
        pp (float): offset pole for PID controller
        w_scheme (np.array): weight array for all optimization schemes.
        nvals (int): number of sampled parameter values for each range

    Returns:
        results (DataFrame): Table of parameterizations and cost values
    """
    n_param = nvals ** 3
    n_scheme = w_scheme.shape[0]
    results = pd.DataFrame(
        np.zeros((n_param,n_scheme+4)),
        columns=['kp','ki','kd','scheme 1','scheme 2','scheme 3','scheme 4','scheme 5','scheme 6','maxpeak']
        )
    param_generator = itertools.product(
        np.linspace(kp_lim[0],kp_lim[1],nvals),
        np.linspace(ki_lim[0],ki_lim[1],nvals),
        np.linspace(kd_lim[0],kd_lim[1],nvals)
    )
    y_max = 0.0
    for idx, (kp, ki, kd) in tqdm(enumerate(param_generator)):
        ctrl = ctrl_PID(kp, ki, kd, pp)
        ts, po, sse, cu, gm, y = cost_PID(plant, ctrl)
        _y_max = y.max()
        y_max = _y_max if _y_max > y_max else y_max
        err = err_PID(ts, po, sse, cu, gm)
        results.at[idx,'kp'] = kp
        results.at[idx,'ki'] = ki
        results.at[idx,'kd'] = kd
        results.at[idx,'maxpeak'] = y.max()
        for s_idx in range(n_scheme):
            results.at[idx,f'scheme {s_idx+1}'] = w_scheme[s_idx] @ err
    return results

# visualization
def plot_result(plant, ctrl, max_peak, title_str=None, ref_dict={}):
    """plot_result

    Args:
        plant (TransferFunction): plant model transfer function
        ctrl (TransferFunction): controller model transfer function
        max_peak (float): upper y value limit for step response plot
        title_str (str, optional): plot title string. Defaults to None.
        ref_dict (dict, optional): reference value dictionary. Defaults to {}.

    Returns:
        _type_: _description_
    """
    sim, _ = sim_PID(plant,ctrl)
    ts_ref = ref_dict.get('ts',TS_REF)
    t = sim.t.squeeze()
    y = sim.y.squeeze()
    fig, ax = plt.subplots(1,1,dpi=150)
    ax.plot(t,y)
    ax.axhline(1) # step response reference goal
    ax.axvline(ts_ref) # settling time reference goal
    ax.set_xlim(0, T_MAX)
    ax.set_ylim(0, max_peak)
    ax.set_xlabel('time (s)')
    ax.set_ylabel('ampl.')
    ax.set_title(title_str)
    return fig, ax

def plot_opt_results(plant,results,pp):
    """plot_opt_results

    Args:
        plant (TransferFunction): plant model transfer function
        results (DataFrame): table of parameterizations and cost values from optimization
        pp (float): PID controller offset parameter

    Returns:
        fig_list ([plt.Figure]): list of figures, one from each optimization scheme
    """
    n_scheme = len([s for s in results.keys() if 'scheme' in s])
    y_max = results['maxpeak'].max()
    fig_list = []
    for s_idx in range(n_scheme):
        scheme_str = f'scheme {s_idx+1}'
        best_idx = results[scheme_str].argmin()
        ctrl = ctrl_PID(
            results.at[best_idx,'kp'],
            results.at[best_idx,'ki'],
            results.at[best_idx,'kd'],
            pp
        )
        fig_title_str = f'{scheme_str} $K_d$: {results.at[best_idx,"kd"]:0.3f} $K_p$: {results.at[best_idx,"kp"]:0.3f} $K_i$: {results.at[best_idx,"ki"]:0.3f} '
        _fig, _ = plot_result(plant,ctrl,y_max,title_str=fig_title_str)
        fig_list.append(_fig)
    return fig_list

def main():
    # create plant
    s = control.tf('s') # laplace variable
    p1 = -1 + 1.5j # define pole
    p2 = p1.conjugate()
    plant = (s+3)/(s*(s-p1)*(s-p2))
    # grid search parameters
    kp_center = 1.4
    ki_center = 0.0
    kd_center = 40.0
    pp = 60.0
    scale_range = 4
    # grid search values
    scale_f = np.sqrt(scale_range)
    kp_lim = (kp_center/scale_f, kp_center*scale_f)
    ki_lim = (ki_center/scale_f, ki_center*scale_f)
    kd_lim = (kd_center/scale_f, kd_center*scale_f)
    # search over grid
    results = grid_search_PID(
        plant,
        kp_lim,
        ki_lim,
        kd_lim,
        pp,
        w_scheme=W_SCHEME,
        nvals=NVALS
    )
    # plot the winners for each error weight scheme
    fig_list = plot_opt_results(plant,results,pp)
    for f_idx, fig in enumerate(fig_list):
        fig.savefig(f'pid_opt_scheme_{f_idx+1}')

if __name__ == "__main__":
    main()