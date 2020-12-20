close all; clear;

q0 = SRD_get('InitialPosition');
v0 = zeros(size(q0));

Handler_State = SRD_get_handler__state('InitialPosition', q0, ...
    'InitialVelocity', zeros(size(q0)));

% % desired trajectory as a function of time - if needed
%
% Handler_IK_Solution = SRD_get('Handler_IK_Solution');
% compount = Handler_IK_Solution.get_position_velocity_acceleration(t);
% q_desired = compount(:, 1);
% v_desired = compount(:, 2);
% a_desired = compount(:, 3);

% %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Handler_dynamics_generalized_coordinates_model = SRD_get('Handler_dynamics_generalized_coordinates_model');


H = Handler_dynamics_generalized_coordinates_model.get_joint_space_inertia_matrix(q0);
c = Handler_dynamics_generalized_coordinates_model.get_bais_vector(q0, v0);
T = Handler_dynamics_generalized_coordinates_model.get_control_map(q0);

u = randn(Handler_dynamics_generalized_coordinates_model.dof_control, 1);

acc = pinv(H) * (T*u - c);

acc