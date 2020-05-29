from selenium import webdriver
from pid import PID
import time
import re


class Parameter:
    def __init__(self, name, dom, target_val, control_keys, tolerance, min_velocity=-float('inf'), max_velocity=float('inf')):
        self.name = name
        self.dom = dom
        self.get_val = lambda: float(re.match('[\+\-.0-9]+', self.dom.text).group(0)) # function to call to get value
        self.target_val = target_val
        self.tolerance = tolerance
        self.control_keys = control_keys
        self.min_velocity = min_velocity
        self.max_velocity = max_velocity

        self.val = self.get_val()
        self.prev_time = time.time()
        self.velocity = 0


class Autopilot:
    def __init__(self, params, pid_controllers, send_key):
        self.params = params
        self.pid = pid_controllers
        self.send_key = send_key
    
    def update_param(self, param):
        cur_val = param.get_val()
        cur_time = time.time()
        velocity = (cur_val - param.val) / (cur_time - param.prev_time)
        param.val = cur_val
        param.prev_time = cur_time
        param.velocity = velocity

        if abs(param.val - param.target_val) < param.tolerance:
            return

        pid_output = self.pid[param.name].update(cur_val)
        if pid_output > 0 and velocity < param.max_velocity:
            self.send_key(param.control_keys[1])
        elif pid_output < 0 and velocity > param.min_velocity:
            self.send_key(param.control_keys[0])
        elif velocity > param.max_velocity + 0.1:
            self.send_key(param.control_keys[0])
        elif velocity < param.min_velocity - 0.1:
            self.send_key(param.control_keys[1])

        if param.name == 'x' and param.val < 5:
            param.min_velocity = -0.2

    def dock(self):
        while True:
            for param in self.params.values():
                self.update_param(param)


def main():
    # connect to the browser
    driver = webdriver.chrome.webdriver.WebDriver()
    driver.get("https://iss-sim.spacex.com/")
    time.sleep(15)
    begin = driver.find_element_by_id("begin-button")
    begin.click()
    time.sleep(10)
    body = driver.find_element_by_tag_name("body")
    # PID parameters
    print('Initializing autopilot...')
    # setup parameters
    param_names = ['x', 'y', 'z', 'pitch', 'yaw', 'roll']
    param_selectors = {'x': '#x-range .distance',
                       'y': '#y-range .distance',
                       'z': '#z-range .distance',
                       'pitch': '#pitch .error',
                       'yaw': '#yaw .error',
                       'roll': '#roll .error'}
    control_keys = {'x': ['e', 'q'],
                    'y': ['a', 'd'],
                    'z': ['s', 'w'],
                    'pitch': [u'\ue015', u'\ue013'],
                    'yaw': [u'\ue014', u'\ue012'],
                    'roll': ['.', ',']}
    gains = {'x': [0.25, 0.1, 3, -0.5, 0.5],
             'y': [0.15, 0.03, 1.5, -0.2, 0.2],
             'z': [0.15, 0.03, 1.5, -0.2, 0.2],
             'pitch': [0.25, 0.01, 1, -0.05, 0.05],
             'yaw': [0.25, 0.01, 1, -0.05, 0.05],
             'roll': [0.25, 0.01, 1, -0.05, 0.05]}
    velocity_range = {'x': (-0.5, 0.5),
                      'y': (-0.5, 0.5),
                      'z': (-0.5, 0.5),
                      'pitch': (-0.5, 0.5),
                      'yaw': (-0.5, 0.5),
                      'roll': (-0.5, 0.5)}
    tolerance = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'pitch': 0.1, 'yaw': 0.1, 'roll': 0.1}
    # initialize pid & autopilot
    params = {}
    pid_controllers = {}
    for param_name in param_names:
        p_gain, i_gain, d_gain, i_term_min, i_term_max = gains[param_name]
        pid_controllers[param_name] = PID(p_gain, i_gain, d_gain, i_term_min, i_term_max)
        min_velocity, max_velocity = velocity_range[param_name]
        param_dom = driver.find_element_by_css_selector(param_selectors[param_name])
        params[param_name] = Parameter(param_name, param_dom, 0, control_keys[param_name], tolerance[param_name], min_velocity, max_velocity)

    send_key = lambda key: body.send_keys(key)
    autopilot = Autopilot(params, pid_controllers, send_key)

    print('Docking started...')
    autopilot.dock()

if __name__ == '__main__':
    main()
