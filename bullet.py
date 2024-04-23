import math
import matplotlib.pyplot as plt

class Bullet:
    def __init__(self, initial_velocity, mass, diameter, drag_coefficient, firing_angle, gun_height=1):
        self.g = 9.81  
        self.air_density = 1.225  
        self.gun_height = gun_height  
        self.initial_velocity = initial_velocity
        self.mass = mass
        self.diameter = diameter
        self.drag_coefficient = drag_coefficient
        self.firing_angle = math.radians(firing_angle)
        self.bullet_area = math.pi * (self.diameter / 2) ** 2
        self.wind_speed = 10  #(m/s)
        self.wind_direction = math.radians(0)  

        # Initial conditions
        self.x = 0  
        self.y = self.gun_height  
        self.v = self.initial_velocity  
        self.θ = self.firing_angle  
        self.t = 0  
        self.dt = 0.01  
        self.flight_time = 0

        self.x_values = [self.x]
        self.y_values = [self.y]
        self.speed_values = [self.v]  

    def calculate_derivatives(self):
        wind_effect = self.wind_speed * math.cos(self.wind_direction - self.θ)

        dxdt = self.v * math.cos(self.θ) + wind_effect
        dydt = self.v * math.sin(self.θ)
        dvdt = -self.g * math.sin(self.θ) - 0.5 * self.air_density * self.v**2 * self.drag_coefficient * self.bullet_area / self.mass
        dθdt = -self.g * math.cos(self.θ) / self.v

        return dxdt, dydt, dvdt, dθdt

    def runge_kutta_step(self):
        k1_x, k1_y, k1_v, k1_θ = self.calculate_derivatives()
        k2_x, k2_y, k2_v, k2_θ = self.calculate_derivatives()
        k3_x, k3_y, k3_v, k3_θ = self.calculate_derivatives()
        k4_x, k4_y, k4_v, k4_θ = self.calculate_derivatives()

        self.x = self.x + self.dt * (k1_x + 2 * k2_x + 2 * k3_x + k4_x) / 6
        self.y = self.y + self.dt * (k1_y + 2 * k2_y + 2 * k3_y + k4_y) / 6
        self.v = self.v + self.dt * (k1_v + 2 * k2_v + 2 * k3_v + k4_v) / 6
        self.θ = self.θ + self.dt * (k1_θ + 2 * k2_θ + 2 * k3_θ + k4_θ) / 6

        self.x_values.append(self.x)
        self.y_values.append(self.y)
        self.speed_values.append(self.v)
        self.t += self.dt

        return self.x, self.y, self.v, self.θ

    def simulate(self, target_distance, target_size):
        while self.y > 0:
            self.runge_kutta_step()
            self.flight_time += self.dt 

            # Check if the bullet hits the target
            if target_distance - target_size/2 <= self.x <= target_distance + target_size/2 and self.y <= target_size:
                delta_t = 0.001  
                delta_p = self.mass * self.v  
                impact_force = delta_p / delta_t  
                print(f"The bullet covered a distance of {self.x} m and hit the target with an impact force of {impact_force} N!")
                break

        if self.y > 0:
            print("Target Missed!")
        else:
            print(f"The bullet covered a distance of {self.x} m before hitting the ground.")

        print(f"Total flight time: {self.flight_time:.2f} seconds")  

        # Plot the trajectory
        plt.figure(figsize=(10, 5))
        plt.plot(self.x_values, self.y_values)
        plt.title('Bullet Trajectory')
        plt.xlabel('Distance (m)')
        plt.ylabel('Height (m)')
        plt.grid(True)
        plt.show()

ak47_bullet = Bullet(initial_velocity=715, mass=0.00792, diameter=0.00762, drag_coefficient=0.03, firing_angle=45)
ak47_bullet.simulate(target_distance=1500, target_size=1.5)
