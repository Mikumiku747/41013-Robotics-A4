%> @file saveJointData.m
%> @author Daniel Selmes
%> @date 2019-10-30

%> @brief Record the data into some globals
function saveJointData(time, q, qVel, error)

    % Globals for data recording
    global data_time;
    global data_q;
    global data_qVel;
    global data_error;
    
    data_time = [data_time; time];
    data_q = [data_q; q];
    data_qVel = [data_qVel; qVel];
    data_error = [data_error; error];
end
