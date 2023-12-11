% Version 1. Se modificó la regla success
%%%%%%%%%%%%%%%% 
% State variables 

state_fluent(success).
state_fluent(right_lane).

%%%%%%%%%%%%%%%% 
% Actions 

action(exec_mdp_right).
action(exec_mdp_left).
action(stop).

%%%%%%%%%%%%%%%% 
% Utilities 

utility(success(1), 1).
utility(correct_mdp(0), 1). 

%%%%%%%%%%%%%%%% 
% Reward model

%success(1) :-  not(rearEnd_crash(0)), not(sideSwipe_crash(0)).

% Exclusive or
correct_mdp(0) :- ((right_lane(0), exec_mdp_right), 
                  not((not(right_lane(0)), exec_mdp_left))); 
                  (not((right_lane(0), exec_mdp_right)), 
                  (not(right_lane(0)), exec_mdp_left)). 

%%%%%%%%%%%%%%%% 
% Transitions

% Stop

1.0::success(1) :- not(success(0)), stop.

% exec_mdp_right

0.75::right_lane(1) :- right_lane(0), exec_mdp_right. % 1 acción de 4 posibles en el carril derecho
0.0::right_lane(1) :-  not(right_lane(0)), exec_mdp_right.

% exec_mdp_right

0.75::right_lane(1) :-  not(right_lane(0)), exec_mdp_left. % 1 acción de 4 posibles en el carril izquierdo 
0.0::right_lane(1) :-  not(right_lane(0)), exec_mdp_left.



