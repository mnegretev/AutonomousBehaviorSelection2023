% Execute: mdp-problog solve -m mdp_left.pl.16 dummy.pl
% It is assumed that obstacle cars on the right lane will move faster than the self-driving car 
    

%%%%%%%%%%%%%%%%%%%%%%%%%% 
% State variables 
%%%%%%%%%%%%%%%%%%%%%%%%%%

state_fluent(free_NW).
state_fluent(free_NE).
state_fluent(free_E).
state_fluent(free_SE).

%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Actions 
%%%%%%%%%%%%%%%%%%%%%%%%%%

action(cruise).
action(keep_distance).
action(change_lane).

%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Utilities 
%%%%%%%%%%%%%%%%%%%%%%%%%%

utility(free_NW(0), 0.5).
utility(free_NE(0), 1.0).
utility(rearEnd_crash, -4.0).  
utility(sideSwipe_crash, -2.0).  
utility(keep_distance, -1.0). 

0.99::rearEnd_crash :- \+ free_NW(0), cruise.
0.99::rearEnd_crash :- \+ free_NE(0), change_lane.
0.95::sideSwipe_crash :- \+ free_E(0), change_lane.

%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Transitions 
%%%%%%%%%%%%%%%%%%%%%%%%%%

% Action: cruise
0.99::free_NW(1) :- free_NW(0), cruise.
0.01::free_NW(1) :- \+ free_NW(0), cruise.
0.80::free_NE(1) :- free_NE(0), cruise.
0.01::free_NE(1) :- \+ free_NE(0), cruise.
0.80::free_E(1) :- free_E(0), cruise.
0.01::free_E(1) :- \+ free_E(0), cruise.
0.80::free_SE(1) :- free_SE(0), cruise.
0.01::free_SE(1) :- \+ free_SE(0), cruise.

% Action: change_lane
0.99::free_NW(1) :- free_NW(0), change_lane.
0.01::free_NW(1) :- \+ free_NW(0), change_lane.
0.99::free_NE(1) :- free_NE(0), free_E(0), (free_SE(0); \+ free_SE(0)),  change_lane.
0.01::free_NE(1) :- (\+ free_NE(0); \+ free_E(0)), change_lane.
0.99::free_E(1) :- free_E(0), change_lane.
0.01::free_E(1) :- \+ free_E(0), change_lane.
0.99::free_SE(1) :- free_SE(0), change_lane.
0.01::free_SE(1) :- \+ free_SE(0), change_lane.

% Action: keep_distance
0.4::free_NW(1) :- \+ free_NW(0), (\+ free_E(0); \+ free_NE(0)), keep_distance.
0.99::free_NE(1) :- free_NE(0), keep_distance.
0.01::free_NE(1) :- \+ free_NE(0), keep_distance.
0.99::free_E(1) :- free_E(0), keep_distance.
0.01::free_E(1) :- \+ free_E(0), keep_distance.
0.99::free_SE(1) :- free_SE(0), keep_distance.
0.01::free_SE(1) :- \+ free_SE(0), keep_distance.


