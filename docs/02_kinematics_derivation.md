# Kinematics Derivation & Mathematical Modeling (4-Wheel Skid-Steer)

> **Course:** Robotics and Intelligent Autonomous Systems (IIIT Dharwad)  
> **Faculty:** Dr. Sunil C K | **Lead:** Raskin Verma  
> **Project:** ASCEND-GROUND (Autonomous Inspection Rover)

---

## 1. 4-Wheel Skid-Steer Mobile Robot Kinematics

The rover uses a **4-wheel skid-steer (differential drive)** mobile platform consisting of four independently driven rubber wheels arranged in a symmetric rectangular layout:
- Front-Left ($FL$) and Rear-Left ($RL$) driven synchronously on the left rail.
- Front-Right ($FR$) and Rear-Right ($RR$) driven synchronously on the right rail.

This layout eliminates fragile caster wheels, prevents tipping, and represents modern industrial inspection rovers (e.g., Clearpath Jackal, SuperDroid, Mars Exploration Rover).

### 1.1 Rover Physical Parameters

| Parameter | Symbol | Value | Units |
|---|---|---|---|
| Wheel radius | $r$ | $0.0425$ | $\text{m}$ ($42.5\,\text{mm}$) |
| Track width (lateral wheel separation) | $L$ | $0.270$ | $\text{m}$ ($270\,\text{mm}$) |
| Wheelbase (longitudinal axle separation) | $B$ | $0.200$ | $\text{m}$ ($200\,\text{mm}$) |
| Chassis dimensions ($l \times w \times h$) | — | $0.35 \times 0.25 \times 0.08$ | $\text{m}$ |
| Total robot mass | $m$ | $2.5$ | $\text{kg}$ |
| Wheel mass (each) | $m_w$ | $0.20$ | $\text{kg}$ |
| Max linear velocity | $v_{\max}$ | $0.5$ | $\text{m/s}$ |
| Max angular velocity | $\omega_{\max}$ | $1.5$ | $\text{rad/s}$ |

---

### 1.2 Forward Kinematics (FK) & Instantaneous Center of Rotation (ICR)

In a 4-wheel skid-steer platform, both left wheels rotate with angular speed $\omega_L$ and both right wheels rotate with angular speed $\omega_R$:
$$\omega_{FL} = \omega_{RL} = \omega_L$$
$$\omega_{FR} = \omega_{RR} = \omega_R$$

The linear surface velocities of the left and right treads are:
$$v_L = r \cdot \omega_L, \qquad v_R = r \cdot \omega_R$$

1. **Forward linear velocity ($v$ along robot longitudinal axis $X_B$):**
$$v = \frac{v_R + v_L}{2} = \frac{r(\omega_R + \omega_L)}{2}$$

2. **Yaw angular velocity ($\omega$ about vertical axis $Z_B$):**
$$\omega = \frac{v_R - v_L}{L_{\text{eff}}} = \frac{r(\omega_R - \omega_L)}{L_{\text{eff}}}$$

where $L_{\text{eff}}$ is the **effective track width** accounting for lateral tire skidding. For pure rolling planar models, $L_{\text{eff}} \approx L = 0.270\,\text{m}$.

In matrix form:
$$\begin{bmatrix} v \\ \omega \end{bmatrix} = \begin{bmatrix} \frac{r}{2} & \frac{r}{2} \\ \frac{r}{L} & -\frac{r}{L} \end{bmatrix} \begin{bmatrix} \omega_R \\ \omega_L \end{bmatrix}$$

Substituting our physical dimensions ($r = 0.0425\,\text{m}$, $L = 0.270\,\text{m}$):
$$\begin{bmatrix} v \\ \omega \end{bmatrix} = \begin{bmatrix} 0.02125 & 0.02125 \\ 0.15741 & -0.15741 \end{bmatrix} \begin{bmatrix} \omega_R \\ \omega_L \end{bmatrix}$$

#### Instantaneous Center of Rotation (ICR)
During a planar turn with angular rate $\omega \neq 0$, the path radius to the robot origin is:
$$R_{\text{turn}} = \frac{v}{\omega} = \frac{L}{2} \cdot \frac{\omega_R + \omega_L}{\omega_R - \omega_L}$$
When $\omega_R = -\omega_L$, $R_{\text{turn}} = 0$ (in-place zero-radius pivot turn).

---

### 1.3 Inverse Kinematics (IK)

Given commanded planar twist $(v, \omega)$ from the navigation controller (`/cmd_vel`):

$$\omega_R = \frac{v + \frac{\omega L}{2}}{r} = \frac{1}{r} v + \frac{L}{2r} \omega$$
$$\omega_L = \frac{v - \frac{\omega L}{2}}{r} = \frac{1}{r} v - \frac{L}{2r} \omega$$

Applied to all four wheels:
$$\omega_{FR} = \omega_{RR} = \omega_R = 23.529\,v + 3.176\,\omega$$
$$\omega_{FL} = \omega_{RL} = \omega_L = 23.529\,v - 3.176\,\omega$$

---

### 1.4 Non-Holonomic Constraint & Wheel Slip Analysis

Because the 4 wheels are parallel and fixed to the chassis, turning requires intentional lateral slippage of the tires relative to the terrain ground contact plane.
- Longitudinal friction coefficient: $\mu_1 = 0.8$ (provides traction and acceleration).
- Lateral friction coefficient: $\mu_2 = 0.1$ (allows skid rotation without motor stall).

The nominal non-holonomic constraint for planar tracking remains:
$$-\sin\theta\,\dot{x} + \cos\theta\,\dot{y} = 0$$

---

### 1.5 Midpoint Odometry Integration

At sampling interval $\Delta t$:
$$\Delta s = v_k \Delta t, \qquad \Delta \theta = \omega_k \Delta t$$

$$\theta_{k+1} = \theta_k + \Delta \theta$$
$$x_{k+1} = x_k + \Delta s \cos\!\left(\theta_k + \frac{\Delta \theta}{2}\right)$$
$$y_{k+1} = y_k + \Delta s \sin\!\left(\theta_k + \frac{\Delta \theta}{2}\right)$$

---

## 2. Sensor Projection & Homogeneous Coordinate Transformation

### 2.1 Pinhole Camera Back-Projection

Given image point $(u, v)$ on $640 \times 480$ sensor with camera matrix:
$$K = \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix}$$

Using range $Z$ from LiDAR bearing correlation:
$$X = \frac{(u - c_x) Z}{f_x}, \qquad Y = \frac{(v - c_y) Z}{f_y}$$

### 2.2 Transform Chain to Map Frame

$$\mathbf{P}^{\text{map}} = T^{\text{map}}_{\text{odom}} \cdot T^{\text{odom}}_{\text{base\_footprint}} \cdot T^{\text{base\_footprint}}_{\text{base\_link}} \cdot T^{\text{base\_link}}_{\text{camera\_link}} \cdot T^{\text{camera\_link}}_{\text{camera\_optical\_frame}} \cdot \begin{bmatrix} X \\ Y \\ Z \\ 1 \end{bmatrix}$$

---

## 3. Pan-Tilt Inspection Mast Derivation (Denavit-Hartenberg)

### 3.1 DH Parameter Table

| Link $i$ | $a_{i-1}$ | $\alpha_{i-1}$ | $d_i$ | $\theta_i$ |
|---|---|---|---|---|
| 1 (Pan) | $0$ | $0^\circ$ | $d_1 = 0.18\,\text{m}$ | $\theta_1^*$ |
| 2 (Tilt) | $0$ | $+90^\circ$ | $0$ | $\theta_2^*$ |
| Cam (Fixed) | $a_2 = 0.05\,\text{m}$ | $0^\circ$ | $0$ | $0^\circ$ |

### 3.2 Forward Kinematics ($T^0_{\text{cam}}$)

$$T^0_{\text{cam}} = \begin{bmatrix} \cos\theta_1\cos\theta_2 & \sin\theta_1 & \cos\theta_1\sin\theta_2 & a_2\cos\theta_1\cos\theta_2 \\ \sin\theta_1\cos\theta_2 & -\cos\theta_1 & \sin\theta_1\sin\theta_2 & a_2\sin\theta_1\cos\theta_2 \\ \sin\theta_2 & 0 & -\cos\theta_2 & d_1 + a_2\sin\theta_2 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

### 3.3 Inverse Kinematics for Defect Targeting

$$\theta_1 = \operatorname{atan2}(y_t, x_t), \qquad \theta_2 = \operatorname{atan2}\!\left(z_t - d_1,\ \sqrt{x_t^2 + y_t^2}\right)$$
