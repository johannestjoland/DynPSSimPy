import numpy as np


def combine_recarrays(a, b):
    new_dtype = a.dtype.descr + b.dtype.descr
    c = np.zeros(a.shape, new_dtype)
    for name in a.dtype.names:
        c[name] = a[name]

    for name in b.dtype.names:
        c[name] = b[name]

    return c


def lookup_strings(a, b):
    # Function to find the index of the element in b that equal the element in a, for each element in a
    if isinstance(a, np.ndarray):
        return np.array([np.where(b == a_)[0][0] for a_ in a])
    else:
        return np.where(b == a)[0][0]


class SimpleRK4:
    def __init__(self, f, t0, x0, t_end, dt=5e-3, **kwargs):
        self.f = f
        self.t = t0
        self.x = x0
        self.y = self.x
        self.t_end = t_end
        self.dt = dt

        for key, value in kwargs.items():
            if key == 'max_step':
                self.dt = value

    def step(self):
        f = self.f
        x = self.x
        t = self.t
        dt = self.dt

        if t < self.t_end:
            k_1 = f(t, x)
            k_2 = f(t + dt / 2, x + (dt / 2) * k_1)
            k_3 = f(t + dt / 2, x + (dt / 2) * k_2)
            k_4 = f(t + dt, x + dt * k_3)

            self.x = x + (dt / 6) * (k_1 + 2 * k_2 + 2 * k_3 + k_4)
            self.t = t + dt
            self.y = self.x
        else:
            print('End of simulation time reached.')


def jacobian_num(f, x, eps=1e-10, **params):
    # Numerical computation of Jacobian
    J = np.zeros([len(x), len(x)], dtype=np.float)

    for i in range(len(x)):
        x1 = x.copy()
        x2 = x.copy()

        x1[i] += eps
        x2[i] -= eps

        f1 = f(x1, **params)
        f2 = f(x2, **params)

        J[:, i] = (f1 - f2) / (2 * eps)

    return J


class DynamicModel:  # This is not used anymore?
    # Empty dummy-class for dynamic models (Gen, AVR, GOV, PSS etc.)
    def __init__(self):
        pass


class EventManager:
    def __init__(self, events, event_function):
        self.events = events
        self.event_flags = np.ones(len(self.events), dtype=bool)
        self.event_function = event_function

    def update(self, t_now):
        for i, (t_event, sub_events) in enumerate(self.events):
            if t_now >= t_event and self.event_flags[i]:
                self.event_flags[i] = False
                for element_type, name, action in sub_events:
                    self.event_function(element_type, name, action)
                    print(name + ' was ' + action + 'ed.')


