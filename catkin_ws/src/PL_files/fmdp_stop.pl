%%%%%%%%%%%%%%%%
% Version 1. 
%%%%%%%%%%%%%%%% 
% State variables 

state_fluent(success).

%%%%%%%%%%%%%%%% 
% Actions 

action(stop).
action(do_nothing).

%%%%%%%%%%%%%%%% 
% Utilities 

utility(continue_trial, 1).
utility(end_trial, 1).

continue_trial :- success(0), do_nothing.
end_trial :- not(success(0)), stop.

%%%%%%%%%%%%%%%% 
% State transitions

0.0::success(1) :- not(success(0)), do_nothing.
1.0::success(1) :- success(0), do_nothing.


