classdef CustomMsgConsts
    %CustomMsgConsts This class stores all message types
    %   The message types are constant properties, which in turn resolve
    %   to the strings of the actual types.
    
    %   Copyright 2014-2017 The MathWorks, Inc.
    
    properties (Constant)
        dobot_magician_GetState = 'dobot_magician/GetState'
        dobot_magician_GetStateRequest = 'dobot_magician/GetStateRequest'
        dobot_magician_GetStateResponse = 'dobot_magician/GetStateResponse'
        dobot_magician_RawCmd = 'dobot_magician/RawCmd'
        dobot_magician_RawCmdRequest = 'dobot_magician/RawCmdRequest'
        dobot_magician_RawCmdResponse = 'dobot_magician/RawCmdResponse'
        dobot_magician_SetPosAng = 'dobot_magician/SetPosAng'
        dobot_magician_SetPosAngRequest = 'dobot_magician/SetPosAngRequest'
        dobot_magician_SetPosAngResponse = 'dobot_magician/SetPosAngResponse'
        dobot_magician_SetPosCart = 'dobot_magician/SetPosCart'
        dobot_magician_SetPosCartRequest = 'dobot_magician/SetPosCartRequest'
        dobot_magician_SetPosCartResponse = 'dobot_magician/SetPosCartResponse'
        dobot_magician_SetPump = 'dobot_magician/SetPump'
        dobot_magician_SetPumpRequest = 'dobot_magician/SetPumpRequest'
        dobot_magician_SetPumpResponse = 'dobot_magician/SetPumpResponse'
        dobot_magician_State = 'dobot_magician/State'
        dobot_ros_EECtrl = 'dobot_ros/EECtrl'
        dobot_ros_EECtrlRequest = 'dobot_ros/EECtrlRequest'
        dobot_ros_EECtrlResponse = 'dobot_ros/EECtrlResponse'
        dobot_ros_GetState = 'dobot_ros/GetState'
        dobot_ros_GetStateRequest = 'dobot_ros/GetStateRequest'
        dobot_ros_GetStateResponse = 'dobot_ros/GetStateResponse'
        dobot_ros_SetPosAng = 'dobot_ros/SetPosAng'
        dobot_ros_SetPosAngRequest = 'dobot_ros/SetPosAngRequest'
        dobot_ros_SetPosAngResponse = 'dobot_ros/SetPosAngResponse'
        dobot_ros_SetPosCart = 'dobot_ros/SetPosCart'
        dobot_ros_SetPosCartRequest = 'dobot_ros/SetPosCartRequest'
        dobot_ros_SetPosCartResponse = 'dobot_ros/SetPosCartResponse'
        dobot_ros_State = 'dobot_ros/State'
    end
    
    methods (Static, Hidden)
        function messageList = getMessageList
            %getMessageList Generate a cell array with all message types.
            %   The list will be sorted alphabetically.
            
            persistent msgList
            if isempty(msgList)
                msgList = cell(20, 1);
                msgList{1} = 'dobot_magician/GetStateRequest';
                msgList{2} = 'dobot_magician/GetStateResponse';
                msgList{3} = 'dobot_magician/RawCmdRequest';
                msgList{4} = 'dobot_magician/RawCmdResponse';
                msgList{5} = 'dobot_magician/SetPosAngRequest';
                msgList{6} = 'dobot_magician/SetPosAngResponse';
                msgList{7} = 'dobot_magician/SetPosCartRequest';
                msgList{8} = 'dobot_magician/SetPosCartResponse';
                msgList{9} = 'dobot_magician/SetPumpRequest';
                msgList{10} = 'dobot_magician/SetPumpResponse';
                msgList{11} = 'dobot_magician/State';
                msgList{12} = 'dobot_ros/EECtrlRequest';
                msgList{13} = 'dobot_ros/EECtrlResponse';
                msgList{14} = 'dobot_ros/GetStateRequest';
                msgList{15} = 'dobot_ros/GetStateResponse';
                msgList{16} = 'dobot_ros/SetPosAngRequest';
                msgList{17} = 'dobot_ros/SetPosAngResponse';
                msgList{18} = 'dobot_ros/SetPosCartRequest';
                msgList{19} = 'dobot_ros/SetPosCartResponse';
                msgList{20} = 'dobot_ros/State';
            end
            
            messageList = msgList;
        end
        
        function serviceList = getServiceList
            %getServiceList Generate a cell array with all service types.
            %   The list will be sorted alphabetically.
            
            persistent svcList
            if isempty(svcList)
                svcList = cell(9, 1);
                svcList{1} = 'dobot_magician/GetState';
                svcList{2} = 'dobot_magician/RawCmd';
                svcList{3} = 'dobot_magician/SetPosAng';
                svcList{4} = 'dobot_magician/SetPosCart';
                svcList{5} = 'dobot_magician/SetPump';
                svcList{6} = 'dobot_ros/EECtrl';
                svcList{7} = 'dobot_ros/GetState';
                svcList{8} = 'dobot_ros/SetPosAng';
                svcList{9} = 'dobot_ros/SetPosCart';
            end
            
            % The message list was already sorted, so don't need to sort
            % again.
            serviceList = svcList;
        end
        
        function actionList = getActionList
            %getActionList Generate a cell array with all action types.
            %   The list will be sorted alphabetically.
            
            persistent actList
            if isempty(actList)
                actList = cell(0, 1);
            end
            
            % The message list was already sorted, so don't need to sort
            % again.
            actionList = actList;
        end
    end
end
