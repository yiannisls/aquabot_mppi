#include <cuda_runtime.h>
#include <math.h>

// Pass all your Goldilocks parameters into this struct
struct MPPIParams {
    int horizon;
    float dt;
    float L, W;
    float m_u, m_v, m_r;
    float d_u, d_v, d_r;
    float d_u_quad, d_v_quad, d_r_quad;
    float q_cross, q_along, q_theta;
    float F_max;

    // Added extra weights from CPU cost function
    float speed_weight;
    float yaw_rate_weight;
    float thrust_weight;
    float steer_weight;
    float target_speed;
};

// This mathematical helper runs directly on the GPU
__device__ float wrap_angle_gpu(float angle) {
    return atan2f(sinf(angle), cosf(angle));
}

// ---------------------------------------------------------
// DYNAMICS DERIVATIVE  (the f(state, control) used by RK4)
// State layout for the 6-vector s: [x, y, yaw, u, v, r]
// Controls are ABSOLUTE actuator commands (already integrated + clamped).
// This is the exact mirror of calculate_dynamics() in motion_node.cpp.
// ---------------------------------------------------------
__device__ void deriv6(
    const float s[6],
    float abs_T_L, float abs_T_R, float abs_a_L, float abs_a_R,
    const MPPIParams& p,
    float ds[6])
{
    float yaw = s[2];
    float u   = s[3];
    float v   = s[4];
    float r   = s[5];

    float F_L = abs_T_L * p.F_max;
    float F_R = abs_T_R * p.F_max;

    float tau_X = (F_L * cosf(abs_a_L)) + (F_R * cosf(abs_a_R));
    float tau_Y = (F_L * sinf(abs_a_L)) + (F_R * sinf(abs_a_R));
    float tau_N = (p.W * 0.5f) * ((F_R * cosf(abs_a_R)) - (F_L * cosf(abs_a_L)))
                - (p.L * 0.5f) * ((F_L * sinf(abs_a_L)) + (F_R * sinf(abs_a_R)));

    float drag_u = (p.d_u * u) + (p.d_u_quad * u * fabsf(u));
    float drag_v = (p.d_v * v) + (p.d_v_quad * v * fabsf(v));
    float drag_r = (p.d_r * r) + (p.d_r_quad * r * fabsf(r));

    // Coriolis-centripetal coupling (3-DOF Fossen, diagonal mass matrix).
    // MUST stay identical to calculate_dynamics() on the CPU.
    // With m_u == m_v the yaw term (cor_r) is exactly zero.
    float cor_u =  p.m_v * v * r;            // sway x yaw -> surge
    float cor_v = -p.m_u * u * r;            // surge x yaw -> sway
    float cor_r =  (p.m_u - p.m_v) * u * v;  // surge x sway -> yaw (0 while m_u==m_v)

    ds[0] = (u * cosf(yaw)) - (v * sinf(yaw));   // x_dot
    ds[1] = (u * sinf(yaw)) + (v * cosf(yaw));   // y_dot
    ds[2] = r;                                   // yaw_dot
    ds[3] = (tau_X - drag_u + cor_u) / p.m_u;    // u_dot
    ds[4] = (tau_Y - drag_v + cor_v) / p.m_v;    // v_dot
    ds[5] = (tau_N - drag_r + cor_r) / p.m_r;    // r_dot
}

// ---------------------------------------------------------
// THE CUDA KERNEL (Runs up to K times simultaneously)
// ---------------------------------------------------------
__global__ void mppi_rollout_kernel(
    const float* initial_state,    // Current boat state [x, y, yaw, u, v, r, T_L, T_R, A_L, A_R]
    const float* nominal_U,        // The current best plan [4 x Horizon]
    const float* noise_samples,    // Pre-generated random noise [K x Horizon x 4]
    const float* reference_path,   // The X, Y, Yaw path points
    float* rollout_costs,          // Output array [K]
    MPPIParams params,
    int K)
{
    // 1. WHICH THREAD AM I?
    int k = blockIdx.x * blockDim.x + threadIdx.x;
    if (k >= K) return;

    // 1. Initialize kinematics into the 6-state vector
    float s[6];
    s[0] = initial_state[0]; // x
    s[1] = initial_state[1]; // y
    s[2] = initial_state[2]; // yaw
    s[3] = initial_state[3]; // u
    s[4] = initial_state[4]; // v
    s[5] = initial_state[5]; // r

    // 2. Initialize ABSOLUTE actuator states
    float abs_T_L = initial_state[6];
    float abs_T_R = initial_state[7];
    float abs_a_L = initial_state[8];
    float abs_a_R = initial_state[9];

    float total_cost = 0.0f;

    for (int h = 0; h < params.horizon; h++) {

        int idx = k * (params.horizon * 4) + (h * 4);
        float n_T_L = noise_samples[idx + 0];
        float n_T_R = noise_samples[idx + 1];
        float n_a_L = noise_samples[idx + 2];
        float n_a_R = noise_samples[idx + 3];

        // These are RATES
        float rate_T_L = nominal_U[h*4 + 0] + n_T_L;
        float rate_T_R = nominal_U[h*4 + 1] + n_T_R;
        float rate_a_L = nominal_U[h*4 + 2] + n_a_L;
        float rate_a_R = nominal_U[h*4 + 3] + n_a_R;

        // Integrate rates to get absolute commands (Euler, ONCE per step,
        // then held constant across the four RK4 stages == zero-order hold).
        // This matches predict_state() exactly.
        abs_T_L += rate_T_L * params.dt;
        abs_T_R += rate_T_R * params.dt;
        abs_a_L += rate_a_L * params.dt;
        abs_a_R += rate_a_R * params.dt;

        // Clamp to physical limits  (thrust floor now 0.01 to MATCH the CPU)
        abs_T_L = fmaxf(0.01f, fminf(abs_T_L, 0.40f));
        abs_T_R = fmaxf(0.01f, fminf(abs_T_R, 0.40f));
        abs_a_L = fmaxf(-0.30f, fminf(abs_a_L, 0.30f));
        abs_a_R = fmaxf(-0.30f, fminf(abs_a_R, 0.30f));

        // ---------- RK4 integration of the 6-state, frozen controls ----------
        float k1[6], k2[6], k3[6], k4[6], tmp[6];

        deriv6(s, abs_T_L, abs_T_R, abs_a_L, abs_a_R, params, k1);
        for (int i = 0; i < 6; i++) tmp[i] = s[i] + 0.5f * params.dt * k1[i];

        deriv6(tmp, abs_T_L, abs_T_R, abs_a_L, abs_a_R, params, k2);
        for (int i = 0; i < 6; i++) tmp[i] = s[i] + 0.5f * params.dt * k2[i];

        deriv6(tmp, abs_T_L, abs_T_R, abs_a_L, abs_a_R, params, k3);
        for (int i = 0; i < 6; i++) tmp[i] = s[i] + params.dt * k3[i];

        deriv6(tmp, abs_T_L, abs_T_R, abs_a_L, abs_a_R, params, k4);
        for (int i = 0; i < 6; i++)
            s[i] += (params.dt / 6.0f) * (k1[i] + 2.0f*k2[i] + 2.0f*k3[i] + k4[i]);

        // Wrap heading and clamp velocities (matches predict_state)
        s[2] = wrap_angle_gpu(s[2]);
        s[3] = fmaxf(-10.0f, fminf(s[3], 10.0f)); // u
        s[4] = fmaxf(-4.0f,  fminf(s[4], 4.0f));  // v
        s[5] = fmaxf(-2.0f,  fminf(s[5], 2.0f));  // r
        // ---------------------------------------------------------------------

        // Cost Calculation
        float ref_x   = reference_path[h*3 + 0];
        float ref_y   = reference_path[h*3 + 1];
        float ref_yaw = reference_path[h*3 + 2];

        float dx = ref_x - s[0];
        float dy = ref_y - s[1];

        // Decompose position error into cross-track (perpendicular to the path)
        // and along-track (parallel to it), relative to the reference heading.
        float e_cross = -sinf(ref_yaw)*dx + cosf(ref_yaw)*dy;
        float e_along =  cosf(ref_yaw)*dx + sinf(ref_yaw)*dy;

        // Huber on cross-track (the error that matters), light quadratic along-track
        float ec = fabsf(e_cross);
        float cross_pen = (ec < 1.0f) ? (ec*ec) : (2.0f*ec - 1.0f);
        float along_pen = e_along * e_along;

        float yaw_err = wrap_angle_gpu(ref_yaw - s[2]);

        // Target Speed Penalty
        float current_speed = hypotf(s[3], s[4]);
        float speed_err = current_speed - params.target_speed;

        // Actuator penalties using ABSOLUTE commands
        float thrust_pen = params.thrust_weight * (abs_T_L*abs_T_L + abs_T_R*abs_T_R);
        float steer_pen  = params.steer_weight  * (abs_a_L*abs_a_L + abs_a_R*abs_a_R);

        total_cost += (params.q_cross * cross_pen) +
                      (params.q_along * along_pen) +
                      (params.q_theta * yaw_err * yaw_err) +
                      (params.speed_weight * speed_err * speed_err) +
                      (params.yaw_rate_weight * s[5] * s[5]) +
                      thrust_pen + steer_pen;
    }

    rollout_costs[k] = total_cost;
}

extern "C" void launch_mppi_cuda(
    float* h_initial_state, float* h_nominal_U, float* h_noise,
    float* h_ref_path, float* h_costs, MPPIParams params, int K)
{
    // 1. Allocate GPU memory (Device memory)
    float *d_state, *d_nom_U, *d_noise, *d_ref, *d_costs;
    cudaMalloc(&d_state, 10 * sizeof(float));
    cudaMalloc(&d_nom_U, params.horizon * 4 * sizeof(float));
    cudaMalloc(&d_noise, K * params.horizon * 4 * sizeof(float));
    cudaMalloc(&d_ref, params.horizon * 3 * sizeof(float));
    cudaMalloc(&d_costs, K * sizeof(float));

    // 2. Copy data from CPU to GPU
    cudaMemcpy(d_state, h_initial_state, 10 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_nom_U, h_nominal_U, params.horizon * 4 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_noise, h_noise, K * params.horizon * 4 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_ref, h_ref_path, params.horizon * 3 * sizeof(float), cudaMemcpyHostToDevice);

    // 3. Launch the Kernel (up to K threads in blocks of 256)
    int threadsPerBlock = 256;
    int blocksPerGrid = (K + threadsPerBlock - 1) / threadsPerBlock;
    mppi_rollout_kernel<<<blocksPerGrid, threadsPerBlock>>>(
        d_state, d_nom_U, d_noise, d_ref, d_costs, params, K);

    // 4. Wait for GPU to finish
    cudaDeviceSynchronize();

    // 5. Copy the K costs back to the CPU
    cudaMemcpy(h_costs, d_costs, K * sizeof(float), cudaMemcpyDeviceToHost);

    // 6. Free GPU memory
    cudaFree(d_state);
    cudaFree(d_nom_U);
    cudaFree(d_noise);
    cudaFree(d_ref);
    cudaFree(d_costs);
}
