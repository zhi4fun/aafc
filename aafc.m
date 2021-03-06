classdef aafc < handle
    % aafc - Adaptive Acceleration Feedforward Control  
    properties 
        % simulation properties
        SimulationStep = 1e5
        SamplingFreq = 348*120
        SamplingTime = 1/(348*120)
        NyquistFreq = 348*60
        % optimal controller
        ThetaD_Optimal_VCM
        ThetaD_Optimal_MA
        ThetaD_VCM
        ThetaD_MA
    end
    properties (Access = private)
        SwitchOffExc = 1e4
        PlotGap = 200
        % transfer functions
        ClosedLoopTF_VCM
        ClosedLoopTF_MA
        EstimateTF_VCM
        EstimateTF_MA
        ControllerTF_VCM
        ControllerTF_MA
        FilterBankTF_VCM
        FilterBankTF_MA
        FilterBankNum
        FilterBankNum_VCM
        FilterBankNum_MA
        % signals
        AccelerationSignal
        ASignal_LowFreq
        ASignal_HighFreq
        DisturbanceSignal
        DSignal_LowFreq
        DSignal_HighFreq
        RROSignal
        NRROSignal
        ExcitationSignal_VCM
        ExcitationSignal_MA
        ControlSignal_VCM
        ControlSignal_MA
        PESSignal
        OptimalControlledPESSignal
        UnControlledPESSignal
        % switch
        RROSwitch = 0
        NRROSwitch = 1
        DisturbanceSwitch = 1
        ExcitationSwitch_VCM = 1
        ExcitationSwitch_MA = 1
        ControlSwitch_VCM = 1
        ControlSwitch_MA = 1
        FilterBankSwitch = 1
        % regressor order
        OrderA_VCM = 5
        OrderB_VCM = 5
        OrderM_VCM = 6
        OrderD_VCM = 6
        OrderA_MA = 5
        OrderB_MA = 5
        OrderM_MA = 6
        OrderD_MA = 6
        % coefficient vector  
        ThetaA_VCM
        ThetaB_VCM
        ThetaM_VCM
        ThetaDF_VCM
        ThetaA_MA
        ThetaB_MA
        ThetaM_MA
        ThetaDF_MA
        % regressor vector
        PhiPES_VCM 
        PhiExcit_VCM
        PhiAccel_VCM
        PhiAccelDelay_VCM
        PhiAccelBorder_VCM
        PhiAccelBfilter_VCM
        PhiPES_MA
        PhiExcit_MA
        PhiAccel_MA
        PhiAccelDelay_MA
        PhiAccelBorder_MA
        PhiAccelBfilter_MA
        % parameters of RLS and Gradiant Descent
        FAB_VCM
        FM_VCM
        FD_VCM
        LambdaAB_VCM
        LambdaM_VCM
        LambdaD_VCM
        FAB_MA
        FM_MA
        FD_MA
        LambdaAB_MA
        LambdaM_MA
        LambdaD_MA
        LearnRate_VCM = 1e-2
        LearnRate_MA = 1e-2
        DecayRate_VCM = 0.95
        DecayRate_MA = 0.95
        StopStep = 1000
        % display
        TM_VCM
        TM_MA
        TD_VCM
        TD_MA
        Bode_opt
    end
    properties (Access = private)
        TFSet
        TFSetOption
        k = 1
        ua_vcm = 0
        ua_ma = 0
        u_vcm = 0
        u_ma = 0
        pes
        uncontrolledpes
        pesf_vcm
        pesf_ma
        excf_vcm
        excf_ma
        accf_vcm
        accf_ma
        pesh_vcm
        pesh_ma
        err_vcm
        err_ma
        pesa_vcm
        pesa_ma
        count = 0
    end
    
    methods % initialization functions
        function AAFC = aafc(SimulationStep)
            if isempty(SimulationStep) == 0
                AAFC.SimulationStep = SimulationStep;
            end
        end
        function initializeSystem(AAFC,varargin)
            % 4 inputs: ClosedLoopTF_VCM
            %           ClosedLoopTF_MA
            %           FilterBankTF_VCM
            %           FilterBankTF_MA
            AAFC.ClosedLoopTF_VCM = varargin{1};
            AAFC.ClosedLoopTF_MA = varargin{2};
            AAFC.FilterBankTF_VCM = varargin{3};
            AAFC.FilterBankTF_MA = varargin{4};
            AAFC.FilterBankNum_VCM = length(varargin{3});
            AAFC.FilterBankNum_MA = length(varargin{4});
            AAFC.FilterBankNum = AAFC.FilterBankNum_VCM + AAFC.FilterBankNum_MA;
        end
        function initializeSignal(AAFC,varargin)
            % 6 inputs: AccelerationSignal
            %           DisturbanceSignal
            %           RROSignal
            %           NRROSignal
            %           ExcitationSignal_VCM
            %           ExcitationSignal_MA
            step = AAFC.SimulationStep;
            AAFC.AccelerationSignal = varargin{1}(1:step);
            AAFC.DisturbanceSignal = varargin{2}(1:step);
            AAFC.RROSignal = varargin{3}(1:step);
            AAFC.NRROSignal = varargin{4}(1:step);
            AAFC.ExcitationSignal_VCM = varargin{5}(1:step);
            AAFC.ExcitationSignal_MA = varargin{6}(1:step);
            AAFC.PESSignal = zeros(1,AAFC.SimulationStep);
            AAFC.OptimalControlledPESSignal = zeros(1,AAFC.SimulationStep);
            AAFC.UnControlledPESSignal = zeros(1,AAFC.SimulationStep);
        end
        function initializeSwitch(AAFC,varargin)
            % 8 inputs: RROSwitch
            %           NRROSwitch
            %           DisturbanceSwitch
            %           ExcitationSwitch_VCM
            %           ExcitationSwitch_MA
            %           ControlSwitch_VCM
            %           ControlSwitch_MA
            %           FilterBankSwitch
            if isempty(varargin) == 0
                AAFC.RROSwitch = varargin{1};
                AAFC.NRROSwitch = varargin{2};
                AAFC.DisturbanceSwitch = varargin{3};
                AAFC.ExcitationSwitch_VCM = varargin{4};
                AAFC.ExcitationSwitch_MA = varargin{5};
                AAFC.ControlSwitch_VCM = varargin{6};
                AAFC.ControlSwitch_MA = varargin{7};
                AAFC.FilterBankSwitch = varargin{8};
            end
        end
        function initializeRegressor(AAFC)
            % initialize regressor vector(zero vector)
            AAFC.PhiPES_VCM = zeros(AAFC.OrderA_VCM,AAFC.FilterBankNum_VCM); 
            AAFC.PhiExcit_VCM = zeros(AAFC.OrderB_VCM,AAFC.FilterBankNum_VCM);
            AAFC.PhiAccel_VCM = zeros(AAFC.OrderD_VCM,1);
            AAFC.PhiAccelDelay_VCM = zeros(AAFC.OrderM_VCM,AAFC.FilterBankNum_VCM);
            AAFC.PhiAccelBorder_VCM = zeros(AAFC.OrderB_VCM,AAFC.FilterBankNum_VCM);
            AAFC.PhiAccelBfilter_VCM = zeros(AAFC.OrderD_VCM,AAFC.FilterBankNum_VCM);
           
            AAFC.PhiPES_MA = zeros(AAFC.OrderA_MA,AAFC.FilterBankNum_MA); 
            AAFC.PhiExcit_MA = zeros(AAFC.OrderB_MA,AAFC.FilterBankNum_MA);
            AAFC.PhiAccel_MA = zeros(AAFC.OrderD_MA,1);
            AAFC.PhiAccelDelay_MA = zeros(AAFC.OrderM_MA,AAFC.FilterBankNum_MA);
            AAFC.PhiAccelBorder_MA = zeros(AAFC.OrderB_MA,AAFC.FilterBankNum_MA);
            AAFC.PhiAccelBfilter_MA = zeros(AAFC.OrderD_MA,AAFC.FilterBankNum_MA);
            % initialize coefficient vector(random vector)
            AAFC.ThetaA_VCM = rand(AAFC.OrderA_VCM,AAFC.FilterBankNum_VCM);
            AAFC.ThetaB_VCM = rand(AAFC.OrderB_VCM,AAFC.FilterBankNum_VCM);
            AAFC.ThetaM_VCM = rand(AAFC.OrderM_VCM,AAFC.FilterBankNum_VCM);
            AAFC.ThetaDF_VCM = rand(AAFC.OrderD_VCM,AAFC.FilterBankNum_VCM);
            AAFC.ThetaD_VCM = rand(AAFC.OrderD_VCM,1);
            
            AAFC.ThetaA_MA = rand(AAFC.OrderA_MA,AAFC.FilterBankNum_MA);
            AAFC.ThetaB_MA = rand(AAFC.OrderB_MA,AAFC.FilterBankNum_MA);
            AAFC.ThetaM_MA = rand(AAFC.OrderM_MA,AAFC.FilterBankNum_MA);
            AAFC.ThetaDF_MA = rand(AAFC.OrderD_MA,AAFC.FilterBankNum_MA);
            AAFC.ThetaD_MA = rand(AAFC.OrderD_MA,1);
            % initialize historical TM TD
            AAFC.TM_VCM = cell(1,AAFC.FilterBankNum_VCM);
            AAFC.TM_VCM(:) = {zeros(AAFC.OrderM_VCM,AAFC.SimulationStep)};
            AAFC.TD_VCM = zeros(AAFC.OrderD_VCM,AAFC.SimulationStep);
            AAFC.TM_MA = cell(1,AAFC.FilterBankNum_MA);
            AAFC.TM_MA(:) = {zeros(AAFC.OrderM_MA,AAFC.SimulationStep)};
            AAFC.TD_MA = zeros(AAFC.OrderD_MA,AAFC.SimulationStep);
        end
        function initializeOrder(AAFC,varargin)
            % 8 inputs: OrderA_VCM
            %           OrderB_VCM
            %           OrderM_VCM
            %           OrderD_VCM
            %           OrderA_MA
            %           OrderB_MA
            %           OrderM_MA
            %           OrderD_MA
            if isempty(varargin) == 0
                AAFC.OrderA_VCM = varargin{1};
                AAFC.OrderB_VCM = varargin{2};
                AAFC.OrderM_VCM = varargin{3};
                AAFC.OrderD_VCM = varargin{4};
                AAFC.OrderA_MA = varargin{5};
                AAFC.OrderB_MA = varargin{6};
                AAFC.OrderM_MA = varargin{7};
                AAFC.OrderD_MA = varargin{8};
            end
            AAFC.initializeRegressor;
        end   
        function initializeRLS(AAFC,varargin)
            % 12 inputs: FAB_VCM
            %           FM_VCM
            %           FD_VCM
            %           LambdaAB_VCM
            %           LambdaM_VCM
            %           LambdaD_VCM
            %           FAB_MA
            %           FM_MA
            %           FD_MA
            %           LambdaAB_MA
            %           LambdaM_MA
            %           LambdaD_MA
            if isempty(varargin) == 0
                AAFC.FAB_VCM = cellfun(@(x) x*eye(AAFC.OrderA_VCM+AAFC.OrderB_VCM),varargin{1},'un',0);
                AAFC.FM_VCM = cellfun(@(x) x*eye(AAFC.OrderM_VCM),varargin{2},'un',0);
                AAFC.FD_VCM = cellfun(@(x) x*eye(AAFC.OrderD_VCM),varargin{3},'un',0);
                AAFC.LambdaAB_VCM = varargin{4};
                AAFC.LambdaM_VCM = varargin{5};
                AAFC.LambdaD_VCM = varargin{6};
                AAFC.FAB_MA = cellfun(@(x) x*eye(AAFC.OrderA_MA+AAFC.OrderB_MA),varargin{7},'un',0);
                AAFC.FM_MA = cellfun(@(x) x*eye(AAFC.OrderM_MA),varargin{8},'un',0);
                AAFC.FD_MA = cellfun(@(x) x*eye(AAFC.OrderD_MA),varargin{9},'un',0);
                AAFC.LambdaAB_MA = varargin{10};
                AAFC.LambdaM_MA = varargin{11};
                AAFC.LambdaD_MA = varargin{12};
            else
                AAFC.FAB_VCM = cell(1,AAFC.FilterBankNum_VCM);
                AAFC.FAB_VCM(:) = {1e8*eye(AAFC.OrderA_VCM+AAFC.OrderB_VCM)};
                AAFC.FM_VCM = cell(1,AAFC.FilterBankNum_VCM);
                AAFC.FM_VCM(:) = {1e6*eye(AAFC.OrderM_VCM)};
                AAFC.FD_VCM = cell(1,AAFC.FilterBankNum_VCM);
                AAFC.FD_VCM(:) = {1e4*eye(AAFC.OrderD_VCM)};
                AAFC.LambdaAB_VCM = cell(1,AAFC.FilterBankNum_VCM);
                AAFC.LambdaAB_VCM(:) = {1-1e-5};
                AAFC.LambdaM_VCM = cell(1,AAFC.FilterBankNum_VCM);
                AAFC.LambdaM_VCM(:) = {1-1e-4};
                AAFC.LambdaD_VCM = cell(1,AAFC.FilterBankNum_VCM);
                AAFC.LambdaD_VCM(:) = {1-1e-4};
                
                AAFC.FAB_MA = cell(1,AAFC.FilterBankNum_MA);
                AAFC.FAB_MA(:) = {1e8*eye(AAFC.OrderA_MA+AAFC.OrderB_MA)};
                AAFC.FM_MA = cell(1,AAFC.FilterBankNum_MA);
                AAFC.FM_MA(:) = {1e6*eye(AAFC.OrderM_MA)};
                AAFC.FD_MA = cell(1,AAFC.FilterBankNum_MA);
                AAFC.FD_MA(:) = {1e4*eye(AAFC.OrderD_MA)};
                AAFC.LambdaAB_MA = cell(1,AAFC.FilterBankNum_MA);
                AAFC.LambdaAB_MA(:) = {1-1e-5};
                AAFC.LambdaM_MA = cell(1,AAFC.FilterBankNum_MA);
                AAFC.LambdaM_MA(:) = {1-1e-4};
                AAFC.LambdaD_MA = cell(1,AAFC.FilterBankNum_MA);
                AAFC.LambdaD_MA(:) = {1-1e-4};
            end
            
        end
        function initializeSGD(AAFC,varargin)
            % 5 inputs: LearnRate_VCM
            %           LearnRate_MA 
            %           DecayRate_VCM
            %           DecayRate_MA 
            %           StopStep
            if isempty(varargin) == 0
                AAFC.LearnRate_VCM = varargin{1};
                AAFC.LearnRate_MA = varargin{2};
                AAFC.DecayRate_VCM = varargin{3};
                AAFC.DecayRate_MA = varargin{4};
                AAFC.StopStep = varargin{5};
            end
        end
        function initializeTFSet(AAFC)
            % initialize states of the set of transfer functions
            AAFC.TFSet = struct();
            AAFC.TFSetOption.storeState = 0;
            AAFC.TFSetOption.storeOutput = 0;
            AAFC.TFSet = AAFC.addstateLTI(AAFC.TFSet,AAFC.ClosedLoopTF_VCM,[],'iir','VCM');
            AAFC.TFSet = AAFC.addstateLTI(AAFC.TFSet,AAFC.ClosedLoopTF_MA,[],'iir','MA');
            for i = 1:AAFC.FilterBankNum_VCM
                AAFC.TFSet = AAFC.addstateLTI(AAFC.TFSet,AAFC.FilterBankTF_VCM{i},[],'fir',['VCM_Filter' num2str(i) '_pes']);
                AAFC.TFSet = AAFC.addstateLTI(AAFC.TFSet,AAFC.FilterBankTF_VCM{i},[],'fir',['VCM_Filter' num2str(i) '_exc']);
                AAFC.TFSet = AAFC.addstateLTI(AAFC.TFSet,AAFC.FilterBankTF_VCM{i},[],'fir',['VCM_Filter' num2str(i) '_acc']);
            end
            for i = 1:AAFC.FilterBankNum_MA
                AAFC.TFSet = AAFC.addstateLTI(AAFC.TFSet,AAFC.FilterBankTF_MA{i},[],'fir',['MA_Filter' num2str(i) '_pes']);
                AAFC.TFSet = AAFC.addstateLTI(AAFC.TFSet,AAFC.FilterBankTF_MA{i},[],'fir',['MA_Filter' num2str(i) '_exc']);
                AAFC.TFSet = AAFC.addstateLTI(AAFC.TFSet,AAFC.FilterBankTF_MA{i},[],'fir',['MA_Filter' num2str(i) '_acc']);
            end
        end
        function initializeSimulation(AAFC)
            AAFC.ua_vcm = 0; 
            AAFC.ua_ma = 0;
            AAFC.u_vcm = 0;
            AAFC.u_ma = 0;
            AAFC.pes = 0;
            AAFC.uncontrolledpes = 0;
            AAFC.pesf_vcm = zeros(1,AAFC.FilterBankNum_VCM);
            AAFC.pesf_ma = zeros(1,AAFC.FilterBankNum_MA);
            AAFC.excf_vcm = zeros(1,AAFC.FilterBankNum_VCM);
            AAFC.excf_ma = zeros(1,AAFC.FilterBankNum_MA);
            AAFC.accf_vcm = zeros(1,AAFC.FilterBankNum_VCM);
            AAFC.accf_ma = zeros(1,AAFC.FilterBankNum_MA);
            AAFC.pesa_vcm = zeros(1,AAFC.FilterBankNum_VCM);
            AAFC.pesa_ma = zeros(1,AAFC.FilterBankNum_MA);
            AAFC.pesh_vcm = zeros(1,AAFC.FilterBankNum_VCM);
            AAFC.pesh_ma = zeros(1,AAFC.FilterBankNum_MA);            
        end
        function initialize(AAFC,CLTF_VCM,CLTF_MA,FBTF_VCM,FBTF_MA,...
                                 Acc,Dist,RRO,NRRO,Ex_VCM,Ex_MA)
            AAFC.initializeSystem(CLTF_VCM,CLTF_MA,FBTF_VCM,FBTF_MA);
            AAFC.initializeSignal(Acc,Dist,RRO,NRRO,Ex_VCM,Ex_MA);
            AAFC.initializeSwitch();
            AAFC.initializeOrder();
            AAFC.initializeRLS();
            AAFC.initializeSGD();
            AAFC.initializeTFSet();
            AAFC.initializeSimulation();
        end
    end
    methods % internal functions
        function generateControlSignal(AAFC,varargin)
            if isempty(varargin) == 0
                AAFC.ExcitationSwitch_VCM = varargin{1};
                AAFC.ExcitationSwitch_MA = varargin{2};
                AAFC.ControlSwitch_VCM = varargin{3};
                AAFC.ControlSwitch_MA = varargin{4};
            end
            a = AAFC.AccelerationSignal(AAFC.k);
            AAFC.PhiAccel_VCM = [a;AAFC.PhiAccel_VCM(1:end-1)];
            AAFC.PhiAccel_MA = [a;AAFC.PhiAccel_MA(1:end-1)];
            AAFC.ua_vcm = AAFC.ThetaD_VCM'*AAFC.PhiAccel_VCM;
            AAFC.ua_ma = AAFC.ThetaD_MA'*AAFC.PhiAccel_MA;
            AAFC.u_vcm = AAFC.ua_vcm*AAFC.ControlSwitch_VCM+...
                         AAFC.ExcitationSignal_VCM(AAFC.k)*AAFC.ExcitationSwitch_VCM;
            AAFC.u_ma = AAFC.ua_ma*AAFC.ControlSwitch_MA+...
                        AAFC.ExcitationSignal_MA(AAFC.k)*AAFC.ExcitationSwitch_MA;
        end
        function generateRealPES(AAFC)
            % generate real PES signal at step k
            [p1,AAFC.TFSet] = AAFC.updatestateLTI(AAFC.TFSet,'VCM',AAFC.u_vcm,AAFC.TFSetOption);
            [p2,AAFC.TFSet] = AAFC.updatestateLTI(AAFC.TFSet,'MA',AAFC.u_ma,AAFC.TFSetOption);
            p0 = AAFC.DisturbanceSignal(AAFC.k)*AAFC.DisturbanceSwitch+...
                 AAFC.RROSignal(AAFC.k)*AAFC.RROSwitch+...
                 AAFC.NRROSignal(AAFC.k)*AAFC.NRROSwitch;
            AAFC.pes = p0 + p1 + p2; 
            AAFC.uncontrolledpes = p0;
        end
        function filterPES(AAFC)
            for i = 1:AAFC.FilterBankNum_VCM
                [AAFC.pesf_vcm(i),AAFC.TFSet] = AAFC.updatestateLTI(AAFC.TFSet,['VCM_Filter' num2str(i) '_pes'],...
                                                               AAFC.pes,AAFC.TFSetOption);
            end
            for i = 1:AAFC.FilterBankNum_MA
                [AAFC.pesf_ma(i),AAFC.TFSet] = AAFC.updatestateLTI(AAFC.TFSet,['MA_Filter' num2str(i) '_pes'],...
                                                              AAFC.pes,AAFC.TFSetOption);
            end
        end
        function filterExcAcc(AAFC)
            for i = 1:AAFC.FilterBankNum_VCM
                [AAFC.excf_vcm(i),AAFC.TFSet] = AAFC.updatestateLTI(AAFC.TFSet,['VCM_Filter' num2str(i) '_exc'],...
                                                               AAFC.ExcitationSignal_VCM(AAFC.k),AAFC.TFSetOption);
                [AAFC.accf_vcm(i),AAFC.TFSet] = AAFC.updatestateLTI(AAFC.TFSet,['VCM_Filter' num2str(i) '_acc'],...
                                                               AAFC.AccelerationSignal(AAFC.k),AAFC.TFSetOption);
            end
            for i = 1:AAFC.FilterBankNum_MA
                [AAFC.excf_ma(i),AAFC.TFSet] = AAFC.updatestateLTI(AAFC.TFSet,['MA_Filter' num2str(i) '_exc'],...
                                                              AAFC.ExcitationSignal_MA(AAFC.k),AAFC.TFSetOption);
                [AAFC.accf_ma(i),AAFC.TFSet] = AAFC.updatestateLTI(AAFC.TFSet,['MA_Filter' num2str(i) '_acc'],...
                                                               AAFC.AccelerationSignal(AAFC.k),AAFC.TFSetOption);
            end
        end
        function estimatePES(AAFC)
            for i = 1:AAFC.FilterBankNum_VCM
                AAFC.pesa_vcm(i) = AAFC.ThetaM_VCM(:,i)'*AAFC.PhiAccelDelay_VCM(:,i);
                AAFC.pesh_vcm(i) = AAFC.ThetaA_VCM(:,i)'*AAFC.PhiPES_VCM(:,i)+...
                                   AAFC.ThetaB_VCM(:,i)'*AAFC.PhiExcit_VCM(:,i)+...
                                   AAFC.pesa_vcm(i);
            end
            for i = 1:AAFC.FilterBankNum_MA
                AAFC.pesa_ma(i) = AAFC.ThetaM_MA(:,i)'*AAFC.PhiAccelDelay_MA(:,i);
                AAFC.pesh_ma(i) = AAFC.ThetaA_MA(:,i)'*AAFC.PhiPES_MA(:,i)+...
                                   AAFC.ThetaB_MA(:,i)'*AAFC.PhiExcit_MA(:,i)+...
                                   AAFC.pesa_ma(i);
            end
        end
        function esterrorPES(AAFC)
            AAFC.err_vcm = AAFC.pesf_vcm-AAFC.pesh_vcm;
            AAFC.err_ma = AAFC.pesf_ma-AAFC.pesh_ma;
        end
        function updatePES(AAFC)
            AAFC.generateRealPES;
            AAFC.filterPES;
            AAFC.filterExcAcc;
            AAFC.estimatePES;
            AAFC.esterrorPES;
        end
        function updateIdentification(AAFC,s)
            if s == 0 % stop updating A and B
                % update System parameters for VCM
                for i = 1:AAFC.FilterBankNum_VCM
                    [AAFC.ThetaM_VCM(:,i),AAFC.FM_VCM{i}] = AAFC.rls(AAFC.ThetaM_VCM(:,i),AAFC.PhiAccelDelay_VCM(:,i),...
                                                                     AAFC.FM_VCM{i},AAFC.err_vcm(i),AAFC.LambdaM_VCM{i});
                end
                % update System parameters for MA
                for i = 1:AAFC.FilterBankNum_MA
                    [AAFC.ThetaM_MA(:,i),AAFC.FM_MA{i}] = AAFC.rls(AAFC.ThetaM_MA(:,i),AAFC.PhiAccelDelay_MA(:,i),...
                                                                   AAFC.FM_MA{i},AAFC.err_ma(i),AAFC.LambdaM_MA{i});
                end  
            else 
                % update System parameters for VCM
                for i = 1:AAFC.FilterBankNum_VCM
                    theta = [AAFC.ThetaA_VCM(:,i);AAFC.ThetaB_VCM(:,i)];
                    phi = [AAFC.PhiPES_VCM(:,i);AAFC.PhiExcit_VCM(:,i)];
                    [theta,AAFC.FAB_VCM{i}] = AAFC.rls(theta,phi,AAFC.FAB_VCM{i},AAFC.err_vcm(i),AAFC.LambdaAB_VCM{i});
                    AAFC.ThetaA_VCM(:,i) = theta(1:AAFC.OrderA_VCM);
                    AAFC.ThetaB_VCM(:,i) = theta(AAFC.OrderA_VCM+1:end);
                    [AAFC.ThetaM_VCM(:,i),AAFC.FM_VCM{i}] = AAFC.rls(AAFC.ThetaM_VCM(:,i),AAFC.PhiAccelDelay_VCM(:,i),...
                                                                     AAFC.FM_VCM{i},AAFC.err_vcm(i),AAFC.LambdaM_VCM{i});
                end
                % update System parameters for MA
                for i = 1:AAFC.FilterBankNum_MA
                    theta = [AAFC.ThetaA_MA(:,i);AAFC.ThetaB_MA(:,i)];
                    phi = [AAFC.PhiPES_MA(:,i);AAFC.PhiExcit_MA(:,i)];
                    [theta,AAFC.FAB_MA{i}] = AAFC.rls(theta,phi,AAFC.FAB_MA{i},AAFC.err_ma(i),AAFC.LambdaAB_MA{i});
                    AAFC.ThetaA_MA(:,i) = theta(1:AAFC.OrderA_MA);
                    AAFC.ThetaB_MA(:,i) = theta(AAFC.OrderA_MA+1:end);
                    [AAFC.ThetaM_MA(:,i),AAFC.FM_MA{i}] = AAFC.rls(AAFC.ThetaM_MA(:,i),AAFC.PhiAccelDelay_MA(:,i),...
                                                                   AAFC.FM_MA{i},AAFC.err_ma(i),AAFC.LambdaM_MA{i});
                end
            end
        end
        function updateControllerLMS(AAFC)
            % apply least mean square (LMS) adaptive filters
            % update learning rate
            AAFC.count = AAFC.count+1;
            if AAFC.count > AAFC.StopStep
                AAFC.count = 1;
                AAFC.LearnRate_VCM = AAFC.LearnRate_VCM*AAFC.DecayRate_VCM;
                AAFC.LearnRate_MA = AAFC.LearnRate_MA*AAFC.DecayRate_MA;
            end
            % update controller parameters for VCM
            x1 = sum(AAFC.PhiAccelBfilter_VCM*diag(AAFC.pesa_vcm),2);
            AAFC.ThetaD_VCM = AAFC.ThetaD_VCM - AAFC.LearnRate_VCM*x1;  
            % updata controller parameters for MA
            x2 = sum(AAFC.PhiAccelBfilter_MA*diag(AAFC.pesa_ma),2);
            AAFC.ThetaD_MA = AAFC.ThetaD_MA - AAFC.LearnRate_MA*x2; 
        end
        function updateControllerNLMS(AAFC)
            % apply normalized least mean square (NLMS) adaptive filters
            % update learning rate
            AAFC.count = AAFC.count+1;
            if AAFC.count > AAFC.StopStep
                AAFC.count = 1;
                AAFC.LearnRate_VCM = AAFC.LearnRate_VCM*AAFC.DecayRate_VCM;
                AAFC.LearnRate_MA = AAFC.LearnRate_MA*AAFC.DecayRate_MA;
            end
            % update controller parameters for VCM
            x1 = sum(AAFC.PhiAccelBfilter_VCM*diag(AAFC.pesa_vcm),2);
            AAFC.ThetaD_VCM = AAFC.ThetaD_VCM - AAFC.LearnRate_VCM*x1/sum(diag(AAFC.PhiAccelBfilter_VCM'*AAFC.PhiAccelBfilter_VCM));  
            % updata controller parameters for MA
            x2 = sum(AAFC.PhiAccelBfilter_MA*diag(AAFC.pesa_ma),2);
            AAFC.ThetaD_MA = AAFC.ThetaD_MA - AAFC.LearnRate_MA*x2/sum(diag(AAFC.PhiAccelBfilter_MA'*AAFC.PhiAccelBfilter_MA));
        end
        function updateControllerRLS(AAFC)
            % apply recursive least square (RLS) adaptive filters
            % update controller parameters for VCM
            for i = 1:AAFC.FilterBankNum_VCM
                [AAFC.ThetaDF_VCM(:,i),AAFC.FD_VCM{i}] = AAFC.rls(AAFC.ThetaDF_VCM(:,i),AAFC.PhiAccelBfilter_VCM(:,i),...
                                                                  AAFC.FD_VCM{i},AAFC.pesa_vcm(i),AAFC.LambdaD_VCM{i});
            end
            AAFC.ThetaD_VCM = sum(-AAFC.ThetaDF_VCM,2);
            % update controller parameters for MA
            for i = 1:AAFC.FilterBankNum_MA
                [AAFC.ThetaDF_MA(:,i),AAFC.FD_MA{i}] = AAFC.rls(AAFC.ThetaDF_MA(:,i),AAFC.PhiAccelBfilter_MA(:,i),...
                                                                AAFC.FD_MA{i},AAFC.pesa_ma(i),AAFC.LambdaD_MA{i});
            end
            AAFC.ThetaD_MA = sum(-AAFC.ThetaDF_MA,2);
        end
        function updateRegressor(AAFC)
            % update VCM regressor
            AAFC.PhiPES_VCM = [AAFC.pesf_vcm;AAFC.PhiPES_VCM(1:end-1,:)];
            AAFC.PhiExcit_VCM = [AAFC.excf_vcm;AAFC.PhiExcit_VCM(1:end-1,:)];
            AAFC.PhiAccelDelay_VCM = [AAFC.accf_vcm;AAFC.PhiAccelDelay_VCM(1:end-1,:)];
            AAFC.PhiAccelBorder_VCM = [AAFC.accf_vcm;AAFC.PhiAccelBorder_VCM(1:end-1,:)];
            for i = 1:AAFC.FilterBankNum_VCM
                x = AAFC.ThetaB_VCM(:,i)'*AAFC.PhiAccelBorder_VCM(:,i);
                AAFC.PhiAccelBfilter_VCM(:,i) = [x;AAFC.PhiAccelBfilter_VCM(1:end-1,i)];
            end
            % update MA regressor
            AAFC.PhiPES_MA = [AAFC.pesf_ma;AAFC.PhiPES_MA(1:end-1,:)];
            AAFC.PhiExcit_MA = [AAFC.excf_ma;AAFC.PhiExcit_MA(1:end-1,:)];
            AAFC.PhiAccelDelay_MA = [AAFC.accf_ma;AAFC.PhiAccelDelay_MA(1:end-1,:)];
            AAFC.PhiAccelBorder_MA = [AAFC.accf_ma;AAFC.PhiAccelBorder_MA(1:end-1,:)];
            for i = 1:AAFC.FilterBankNum_MA
                x = AAFC.ThetaB_MA(:,i)'*AAFC.PhiAccelBorder_MA(:,i);
                AAFC.PhiAccelBfilter_MA(:,i) = [x;AAFC.PhiAccelBfilter_MA(1:end-1,i)];
            end
        end
        function storeSignal(AAFC)
            % store PES signal
            AAFC.PESSignal(AAFC.k) = AAFC.pes;
            AAFC.UnControlledPESSignal(AAFC.k) = AAFC.uncontrolledpes;
            % store historical TM TD
            for i = 1:AAFC.FilterBankNum_VCM
                AAFC.TM_VCM{i}(:,AAFC.k) = AAFC.ThetaM_VCM(:,i);
            end
            AAFC.TD_VCM(:,AAFC.k) = AAFC.ThetaD_VCM;
            for i = 1:AAFC.FilterBankNum_MA
                AAFC.TM_MA{i}(:,AAFC.k) = AAFC.ThetaM_MA(:,i);
            end
            AAFC.TD_MA(:,AAFC.k) = AAFC.ThetaD_MA;
        end
    end
    methods % optimal control functions
        function filterSignal(AAFC,varargin)
            % 2 FIR filters: low-pass and high-pass
            if isempty(varargin) == 0
                LowPassFilter = varargin{1};
                HighPassFilter = varargin{2};
            else
                LowPassFilter = 0;
                HighPassFilter = 0;
                for i = 1:AAFC.FilterBankNum_VCM
                    LowPassFilter = LowPassFilter+AAFC.FilterBankTF_VCM{i};
                end
                for i = 1:AAFC.FilterBankNum_MA
                    HighPassFilter = HighPassFilter+AAFC.FilterBankTF_MA{i};
                end
            end
            AAFC.DSignal_LowFreq = filter(LowPassFilter,1,AAFC.DisturbanceSignal);
            AAFC.ASignal_LowFreq = filter(LowPassFilter,1,AAFC.AccelerationSignal);
            AAFC.DSignal_HighFreq = filter(HighPassFilter,1,AAFC.DisturbanceSignal);
            AAFC.ASignal_HighFreq = filter(HighPassFilter,1,AAFC.AccelerationSignal);
        end
        function optimalD_VCM(AAFC,varargin)
            % find the optimal controller parameters for VCM only
            if isempty(varargin) == 0
                n = varargin;
            else 
                n = AAFC.SimulationStep;
            end
            B = AAFC.ClosedLoopTF_VCM.Numerator{1};
            A = AAFC.ClosedLoopTF_VCM.Denominator{1};
            Pd = AAFC.DSignal_LowFreq(1:n)';
            Pa = [zeros(1,AAFC.OrderD_VCM-1),filter(B,A,AAFC.ASignal_LowFreq(1:n))];
            Ma = zeros(n,AAFC.OrderD_VCM);
            for i = 1:n
                Ma(i,:) = flip(Pa(i:i+AAFC.OrderD_VCM-1));
            end
            AAFC.ThetaD_Optimal_VCM = (Ma'*Ma)\(-Ma'*Pd);
        end
        function optimalD_MA(AAFC,varargin)
            % find the optimal controller parameters for MA only
            if isempty(varargin) == 0
                n = varargin;
            else 
                n = AAFC.SimulationStep;
            end
            B = AAFC.ClosedLoopTF_MA.Numerator{1};
            A = AAFC.ClosedLoopTF_MA.Denominator{1};
            Pd = AAFC.DSignal_HighFreq(1:n)';
            Pa = [zeros(1,AAFC.OrderD_MA-1),filter(B,A,AAFC.ASignal_HighFreq(1:n))];
            Ma = zeros(n,AAFC.OrderD_MA);
            for i = 1:n
                Ma(i,:) = flip(Pa(i:i+AAFC.OrderD_MA-1));
            end
            AAFC.ThetaD_Optimal_MA = (Ma'*Ma)\(-Ma'*Pd);
        end
        function optimalD_Dual(AAFC,varargin)
            % find the optimal controller parameters by from dual-actuators
            if isempty(varargin) == 0
                n = varargin;
            else 
                n = AAFC.SimulationStep;
            end
            B1 = AAFC.ClosedLoopTF_VCM.Numerator{1};
            A1 = AAFC.ClosedLoopTF_VCM.Denominator{1};
            B2 = AAFC.ClosedLoopTF_MA.Numerator{1};
            A2 = AAFC.ClosedLoopTF_MA.Denominator{1};
            Pd = AAFC.DisturbanceSignal(1:n)';
            Pa1 = [zeros(1,AAFC.OrderD_VCM-1),filter(B1,A1,AAFC.AccelerationSignal(1:n))];
            Pa2 = [zeros(1,AAFC.OrderD_MA-1),filter(B2,A2,AAFC.AccelerationSignal(1:n))];
            Ma1 = zeros(n,AAFC.OrderD_VCM);
            Ma2 = zeros(n,AAFC.OrderD_MA);
            for i = 1:n
                Ma1(i,:) = flip(Pa1(i:i+AAFC.OrderD_VCM-1));
                Ma2(i,:) = flip(Pa2(i:i+AAFC.OrderD_MA-1));
            end
            M = Ma2' - Ma2'*Ma1*((Ma1'*Ma1)\Ma1');
            AAFC.ThetaD_Optimal_MA = (M*Ma2)\(-M*Pd);
            AAFC.ThetaD_Optimal_VCM = (Ma1'*Ma1)\(-Ma1'*Ma2*AAFC.ThetaD_Optimal_MA-Ma1'*Pd);
        end
        function optimalControl(AAFC)
            AAFC.initializeRegressor;
            AAFC.initializeSimulation;
            AAFC.ThetaD_VCM = AAFC.ThetaD_Optimal_VCM;
            AAFC.ThetaD_MA = AAFC.ThetaD_Optimal_MA;
            AAFC.k = 1; 
            while AAFC.k <= AAFC.SimulationStep
                AAFC.generateControlSignal(0,0,1,1); % only compensation
                AAFC.generateRealPES;
                AAFC.OptimalControlledPESSignal(AAFC.k) = AAFC.pes;
                AAFC.UnControlledPESSignal(AAFC.k) = AAFC.uncontrolledpes;            
                AAFC.k = AAFC.k+1;
            end
            disp('Achieved the optimal compensation performance.');
        end      
    end
    methods % adaptive control functions
        function adaptiveControlLMS(AAFC)
            AAFC.initializeRegressor;
            AAFC.initializeTFSet;
            AAFC.initializeSimulation;
            AAFC.k = 1; 
            n = 1;
            AAFC.count = 0;
            while AAFC.k <= AAFC.SimulationStep
                if AAFC.k <= AAFC.SwitchOffExc
                    AAFC.generateControlSignal(1,1,0,0); % only excitation
                    AAFC.updatePES;
                    AAFC.updateIdentification(1); % update A and B
                    AAFC.updateRegressor;
                    AAFC.storeSignal;
                else
                    AAFC.generateControlSignal(0,0,1,1); % only compensation
                    AAFC.updatePES;
                    AAFC.updateIdentification(0); % stop updating A and B
                    AAFC.updateControllerLMS; % update D
                    AAFC.updateRegressor;
                    AAFC.storeSignal;
                end
                if n >= 5000, disp(int2str(AAFC.k)); n = 0; end
                n = n+1;
                AAFC.k = AAFC.k+1;
            end
        end
        function adaptiveControlNLMS(AAFC)
            AAFC.initializeRegressor;
            AAFC.initializeTFSet;
            AAFC.initializeSimulation;
            AAFC.k = 1; 
            n = 1;
            AAFC.count = 0;
            while AAFC.k <= AAFC.SimulationStep
                if AAFC.k <= AAFC.SwitchOffExc
                    AAFC.generateControlSignal(1,1,0,0); % only excitation
                    AAFC.updatePES;
                    AAFC.updateIdentification(1); % update A and B
                    AAFC.updateRegressor;
                    AAFC.storeSignal;
                else
                    AAFC.generateControlSignal(0,0,1,1); % only compensation
                    AAFC.updatePES;
                    AAFC.updateIdentification(0); % stop updating A and B
                    AAFC.updateControllerNLMS; % update D
                    AAFC.updateRegressor;
                    AAFC.storeSignal;
                end
                if n >= 5000, disp(int2str(AAFC.k)); n = 0; end               
                n = n+1;
                AAFC.k = AAFC.k+1;
            end
        end
        function adaptiveControlRLS(AAFC)
            AAFC.initializeRegressor;
            AAFC.initializeTFSet;
            AAFC.initializeSimulation;
            AAFC.k = 1; 
            n = 1;
            AAFC.count = 0;
            while AAFC.k <= AAFC.SimulationStep
                if AAFC.k <= AAFC.SwitchOffExc
                    AAFC.generateControlSignal(1,1,0,0); % only excitation
                    AAFC.updatePES;
                    AAFC.updateIdentification(1); % update A and B
                    AAFC.updateRegressor;
                    AAFC.storeSignal;
                else
                    AAFC.generateControlSignal(0,0,1,1); % only compensation
                    AAFC.updatePES;
                    AAFC.updateIdentification(0); % stop updating A and B
                    AAFC.updateControllerRLS; % update D
                    AAFC.updateRegressor;
                    AAFC.storeSignal;
                end
                if n >= 5000, disp(int2str(AAFC.k)); n = 0; end
                n = n+1;
                AAFC.k = AAFC.k+1;
            end
        end
    end
    methods % plot functions
        function plotPES(AAFC)
            figure; plot(AAFC.UnControlledPESSignal); hold on;
                    plot(AAFC.PESSignal);
            figure; AAFC.fftp(AAFC.UnControlledPESSignal(end-1e4:end),AAFC.SamplingFreq); hold on;
                    AAFC.fftp(AAFC.PESSignal(end-1e4:end),AAFC.SamplingFreq);
            figure; histogram(AAFC.UnControlledPESSignal(end-1e4:end),'Normalization','probability','BinWidth',0.025); hold on;
                    histogram(AAFC.PESSignal(end-1e4:end),'Normalization','probability','BinWidth',0.025);
        end
        function plotMinPES(AAFC)
            figure; plot(AAFC.UnControlledPESSignal); hold on;
                    plot(AAFC.OptimalControlledPESSignal);
            figure; AAFC.fftp(AAFC.UnControlledPESSignal(end-1e4:end),AAFC.SamplingFreq); hold on;
                    AAFC.fftp(AAFC.OptimalControlledPESSignal(end-1e4:end),AAFC.SamplingFreq);
            figure; histogram(AAFC.UnControlledPESSignal,'Normalization','probability','BinWidth',0.025); hold on;
                    histogram(AAFC.OptimalControlledPESSignal,'Normalization','probability','BinWidth',0.025);
        end
        function plotAcc(AAFC)
            figure; plot(AAFC.AccelerationSignal);
            figure; AAFC.fftp(AAFC.AccelerationSignal(end-1e4:end),AAFC.SamplingFreq);
        end
        function plotDist(AAFC)
            figure; plot(AAFC.DisturbanceSignal);
            figure; AAFC.fftp(AAFC.DisturbanceSignal(end-1e4:end),AAFC.SamplingFreq);
        end
        function plotVCM(AAFC,Bode_Opt)
            for i = 1:AAFC.FilterBankNum_VCM
                est_VCM = tf(AAFC.ThetaB_VCM(:,i)',[1,-AAFC.ThetaA_VCM(:,i)'],AAFC.SamplingTime);
                figure; bode(AAFC.ClosedLoopTF_VCM,est_VCM,Bode_Opt);
            end
        end
        function plotMA(AAFC,Bode_Opt)
            for i = 1:AAFC.FilterBankNum_MA
                est_MA = tf(AAFC.ThetaB_MA(:,i)',[1,-AAFC.ThetaA_MA(:,i)'],AAFC.SamplingTime);
                figure; bode(AAFC.ClosedLoopTF_MA,est_MA,Bode_Opt);
            end
        end
        function plotTM_VCM(AAFC)
            K = repmat(1:AAFC.PlotGap:AAFC.SimulationStep,AAFC.OrderM_VCM,1);
            for i = 1:AAFC.FilterBankNum_VCM
                figure; plot(K,AAFC.TM_VCM{i}(:,1:AAFC.PlotGap:end),'.b'); grid on;
            end
        end
        function plotTM_MA(AAFC)
            K = repmat(1:AAFC.PlotGap:AAFC.SimulationStep,AAFC.OrderM_MA,1);
            for i = 1:AAFC.FilterBankNum_MA
                figure; plot(K,AAFC.TM_MA{i}(:,1:AAFC.PlotGap:end),'.b'); grid on;
            end
        end
        function plotTD_VCM(AAFC)
            K = repmat(1:AAFC.PlotGap:AAFC.SimulationStep,AAFC.OrderD_VCM,1);
            figure; plot(K,AAFC.TD_VCM(:,1:AAFC.PlotGap:end),'.b'); grid on;
        end
        function plotTD_MA(AAFC)
            K = repmat(1:AAFC.PlotGap:AAFC.SimulationStep,AAFC.OrderD_MA,1);
            figure; plot(K,AAFC.TD_MA(:,1:AAFC.PlotGap:end),'.b'); grid on;
        end
    end
    methods(Static)
        function plant = addstateLTI(plant,sys,x0,type,name)
            % add states of LTI-system
            if nargin < 4, type = 'ss'; end

            if ( (nargin <=2) && strcmp(type,'ss') )
                if isempty(sys.A)%Static gain
                x0 = 0;
                else
                x0 = zeros(size(sys.A,1),1);
                end
            end
    
            if strcmpi(type, 'ss')
                if ( (sys.InputDelay + sys.OutputDelay  ~= 0) || ( ~isempty(sys.InternalDelay) && (sys.InternalDelay ~=0) ) )
                    error('The input system should have zero Input/Output/Internal delay. If the system has delays, they should be modeled by adding delayed states in the state vector.');
                end
                if isempty(sys.a), sys.a = 0; end
                if isempty(sys.b), sys.b = 0; end
                if isempty(sys.c), sys.c = 0; end
                if isempty(sys.d), sys.d = 0; end
                plant.(name) = struct('x',x0,'y',[],'a',sys.a,'b',sys.b,'c',sys.c,'d',sys.d,'Ts',sys.Ts, 'type', type);
            elseif strcmpi(type, 'iir')
                [b,a] = tfdata(sys, 'v');
                denDeg = length(pole(sys));
                numDeg = length(zero(sys));
                relDeg = denDeg - numDeg;
                b = b(relDeg+1:end);
                a = a(2:end); %remove 1 from a
                w = [a b];
                if isempty(x0)
                    x0 = zeros(length(w),1);
                end
                [numInp,~] = size(x0);
                if numInp == length(w);
                    Ur = zeros(relDeg,1);
                else
                    Ur = zeros(numInp,relDeg);
                end
                Ts = sys.Ts;
                plant.(name) = struct('x', x0, 'y',[],'w',[a b]', 'numDeg', numDeg, 'denDeg', denDeg,'Ts',Ts, 'type',type,'W',[],'Ur',Ur);
            elseif strcmpi(type, 'fir')
                if ( ~isvector(sys) )
                    error( 'Second argument, in2, should be a vector such that in2(i) gives the coefficient for u(k-i)');
                end
                sys = reshape(sys,[],1);
                if isempty(x0)
                    x0 = zeros(length(sys),1);
                end
                plant.(name) = struct('x', x0, 'y',[],'w',sys, 'type',type,'W',[]);
            end
        end
        function [output,plant] = updatestateLTI(plant,sysName,input,opt)
            % update states of LTI-system
            sysStruct = plant.(sysName);
            x = sysStruct.x;

            if strcmpi(sysStruct.type, 'ss')
                xNext = sysStruct.a * x(:,end) + sysStruct.b * input;
                output = sysStruct.c * x(:,end) + sysStruct.d * input;
            elseif strcmpi(sysStruct.type, 'iir')
                denDeg = sysStruct.denDeg;
                Unew = [sysStruct.Ur(end); x(denDeg+1:end-1, end)]; % [u(k) u(k-1) ... u(k-denDeg)]
                sysStruct.Ur = [input;sysStruct.Ur(1:end-1)];
                Y = x(1:denDeg,end); % [-y(k-1) -y(k-2) ... -y(k-denDeg)]
                output = [Y; Unew]' * sysStruct.w;
                Ynew = [-output; Y(1:denDeg-1)];
                xNext = [Ynew ; Unew];
            elseif strcmpi(sysStruct.type, 'fir')
                % by default FIR = w0 + w1*z^-1 + ...., w0 !=0
                xNext = [input; x(1:end-1, end) ];
                output = xNext(:,end)' * sysStruct.w;
            end
    
            if opt.storeState ~= 0
                sysStruct.x = [x xNext];
            else
                sysStruct.x = xNext;
            end
    
            if opt.storeOutput ~= 0
                sysStruct.y = [sysStruct.y output];
            else
                sysStruct.y = output;
            end
            plant.(sysName) = sysStruct;
        end
        function [theta,F] = rls(theta,phi,F,err,lambda)
            % recursive least square filter
            g = F*phi/(lambda+phi'*F*phi);
            theta = theta+err*g;
            F = 1/lambda*(F-g*phi'*F);
        end
        function fftp(y,Fs,NFFT)
            % fft and plot the spectrum of signal 
            if nargin < 2
                Fs = 2*pi;
            end
            L = numel(y);
            if nargin < 3
                NFFT = 2^nextpow2(L); % Next power of 2 from length of y
            end
            X = fft(y,NFFT)/L;
            f = Fs/2*linspace(0,1,NFFT/2+1);
            % Plot single-sided amplitude spectrum.
            magabs = 2*abs(X(1:NFFT/2+1));
            magdb = 20*log10(magabs);
            plot(f,magabs); 
            title('Single-Sided Amplitude Spectrum of y(t)')
            xlabel('Frequency (Hz)')
            ylabel('|Y(f)|')
        end
    end
end