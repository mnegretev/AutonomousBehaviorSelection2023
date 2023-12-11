% Execute: mdp-problog solve -m mdp_right.pl dummy.pl
% It is assumed that obstacle cars on the left lane will move faster than the self-drivng car. 

%%%%%%%%%%%%%%%%%%%%%%%%%% 
% State variables 
%%%%%%%%%%%%%%%%%%%%%%%%%%

state_fluent(free_NE).
state_fluent(free_NW).
state_fluent(free_SW).
state_fluent(free_W).

%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Actions 
%%%%%%%%%%%%%%%%%%%%%%%%%%

action(cruise).
action(keep_distance).
action(change_lane).

%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Utilities 
%%%%%%%%%%%%%%%%%%%%%%%%%%

utility(free_NE(0), 1.0).
utility(free_NW(0), 0.1).
utility(rearEnd_crash, -4.0).  
utility(sideSwipe_crash, -2.0).  
utility(keep_distance, -1.0). 

0.99::rearEnd_crash :- \+ free_NE(0), cruise.
0.99::rearEnd_crash :- \+ free_NW(0), change_lane.
0.95::sideSwipe_crash :- \+ free_W(0), change_lane.

%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Transitions 
%%%%%%%%%%%%%%%%%%%%%%%%%%

% Action: cruise
0.99::free_NE(1) :- free_NE(0), cruise.
0.01::free_NE(1) :- \+ free_NE(0), cruise.
0.99::free_NW(1) :- free_NW(0), cruise.
0.01::free_NW(1) :- \+ free_NW(0), cruise.
0.99::free_W(1) :- free_W(0), cruise.
0.01::free_W(1) :- \+ free_W(0), cruise.
0.99::free_SW(1) :- free_SW(0), cruise.
0.01::free_SW(1) :- \+ free_SW(0), cruise.

% Action: change_lane
0.99::free_NE(1) :- \+ free_NE(0), free_NW(0), free_SW(0), free_W(0),  change_lane.
0.01::free_NE(1) :- \+ free_NE(0), (\+ free_NW(0); \+ free_W(0); \+ free_SW(0)), change_lane.
0.99::free_NW(1) :- free_NW(0), change_lane.
0.01::free_NW(1) :- \+ free_NW(0), change_lane.
0.99::free_W(1) :- free_W(0), change_lane.
0.01::free_W(1) :- \+ free_W(0), change_lane.
0.99::free_SW(1) :- free_SW(0), change_lane.
0.01::free_SW(1) :- \+ free_SW(0), change_lane.

% Action: keep_distance
0.5::free_NE(1) :- \+ free_NE(0), (\+ free_W(0); \+ free_NW(0); \+ free_SW(0)), keep_distance.
0.99::free_NW(1) :- free_NW(0), keep_distance.
0.01::free_NW(1) :- \+ free_NW(0), keep_distance.
0.99::free_W(1) :- free_W(0), keep_distance.
0.01::free_W(1) :- \+ free_W(0), keep_distance.
0.99::free_SW(1) :- free_SW(0), keep_distance.
0.01::free_SW(1) :- \+ free_SW(0), keep_distance.

