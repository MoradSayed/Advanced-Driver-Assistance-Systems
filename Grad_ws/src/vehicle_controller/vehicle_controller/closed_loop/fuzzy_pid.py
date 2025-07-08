import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

class Fuzzy:
    def __init__(self):
        # Define universe ranges for input (error, delta_error) and output (Kp, Ki, Kd modifiers)
        self.error = ctrl.Antecedent(np.linspace(-1, 1, 5), 'error')
        self.delta_error = ctrl.Antecedent(np.linspace(-1, 1, 5), 'delta_error')

        self.kp_mod = ctrl.Consequent(np.linspace(0.5, 1.5, 5), 'kp_mod')
        self.ki_mod = ctrl.Consequent(np.linspace(0.5, 1.5, 5), 'ki_mod')
        self.kd_mod = ctrl.Consequent(np.linspace(0.5, 1.5, 5), 'kd_mod')

        # Membership functions
        names = ['NB', 'NS', 'ZE', 'PS', 'PB']
        for var in [self.error, self.delta_error]:
            var.automf(names=names)

        for mod in [self.kp_mod, self.ki_mod, self.kd_mod]:
            mod['NB'] = fuzz.trimf(mod.universe, [0.5, 0.5, 0.75])
            mod['NS'] = fuzz.trimf(mod.universe, [0.5, 0.75, 1.0])
            mod['ZE'] = fuzz.trimf(mod.universe, [0.9, 1.0, 1.1])  # Default neutral state
            mod['PS'] = fuzz.trimf(mod.universe, [1.0, 1.25, 1.5])
            mod['PB'] = fuzz.trimf(mod.universe, [1.25, 1.5, 1.5])

        # Refined rule base for ACC system
        kp_rules = [
            ctrl.Rule(self.error['NB'] & self.delta_error['NB'], self.kp_mod['PB']),
            ctrl.Rule(self.error['NS'] & self.delta_error['NS'], self.kp_mod['PS']),
            ctrl.Rule(self.error['PS'] & self.delta_error['PS'], self.kp_mod['PS']),
            ctrl.Rule(self.error['PB'] & self.delta_error['PB'], self.kp_mod['PB']),
            ctrl.Rule(self.error['ZE'] & self.delta_error['ZE'], self.kp_mod['ZE']),
        ]
        ki_rules = [
            ctrl.Rule(self.error['NB'] & self.delta_error['NB'], self.ki_mod['NS']),
            ctrl.Rule(self.error['NS'] & self.delta_error['NS'], self.ki_mod['NS']),
            ctrl.Rule(self.error['PS'] & self.delta_error['PS'], self.ki_mod['PS']),
            ctrl.Rule(self.error['PB'] & self.delta_error['PB'], self.ki_mod['PS']),
            ctrl.Rule(self.error['ZE'] & self.delta_error['ZE'], self.ki_mod['ZE']),
        ]
        kd_rules = [
            ctrl.Rule(self.error['NB'] & self.delta_error['NB'], self.kd_mod['PB']),
            ctrl.Rule(self.error['NS'] & self.delta_error['NS'], self.kd_mod['PS']),
            ctrl.Rule(self.error['PS'] & self.delta_error['PS'], self.kd_mod['NS']),
            ctrl.Rule(self.error['PB'] & self.delta_error['PB'], self.kd_mod['PB']),
            ctrl.Rule(self.error['ZE'] & self.delta_error['ZE'], self.kd_mod['ZE']),
        ]

        # Create control systems
        self.kp_ctrl = ctrl.ControlSystem(kp_rules)
        self.ki_ctrl = ctrl.ControlSystem(ki_rules)
        self.kd_ctrl = ctrl.ControlSystem(kd_rules)

        self.kp_sim = ctrl.ControlSystemSimulation(self.kp_ctrl)
        self.ki_sim = ctrl.ControlSystemSimulation(self.ki_ctrl)
        self.kd_sim = ctrl.ControlSystemSimulation(self.kd_ctrl)

    def compute(self, error: float, max_error: float, delta_error: float, max_delta_error: float = 10.0) -> float:
        #? Normalize error and delta_error
        normalized_error = np.clip(error / max_error, -1, 1)
        normalized_delta_error = np.clip(delta_error / max_delta_error, -1, 1)

        # Update fuzzy modifiers with normalized values
        self.kp_sim.input['error'] = normalized_error
        self.kp_sim.input['delta_error'] = normalized_delta_error
        self.ki_sim.input['error'] = normalized_error
        self.ki_sim.input['delta_error'] = normalized_delta_error
        self.kd_sim.input['error'] = normalized_error
        self.kd_sim.input['delta_error'] = normalized_delta_error

        # Modulated gains
        try:
            self.kp_sim.compute()
            Kp_mod = self.kp_sim.output['kp_mod']
        except:
            Kp_mod = 1
        try:
            self.ki_sim.compute()
            Ki_mod = self.ki_sim.output['ki_mod']
        except:
            Ki_mod = 1
        try:
            self.kd_sim.compute()
            Kd_mod = self.kd_sim.output['kd_mod']
        except:
            Kd_mod = 1

        return Kp_mod, Ki_mod, Kd_mod
