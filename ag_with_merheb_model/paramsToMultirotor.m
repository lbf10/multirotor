function multirotor = paramsToMultirotor(attitudeController, controlAllocator, attitudeReference, multirotor, x)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    pKp = x(1:3);
    pKi = x(4:6);
    pKd = x(7:9);
    pKdd = x(10:12);
    multirotor.configController('Position PIDD',pKp,pKi,pKd,pKdd);
    
    switch attitudeController
        case 'PID'
            kp = x(13:15);
            ki = x(16:18);
            kd = x(19:21);
            multirotor.configController(attitudeController,kp,ki,kd);
            index = 22;
        case 'RLQ-R Passive'
            P = diag(x(13:18));
            Q = diag(x(19:24));
            R = diag(x(25:27));
            Ef = reshape(x(28:33),[1 6]);
            Eg = reshape(x(34:36),[1 3]);
            H = reshape(x(37:42),[6 1]);
            mu = x(43);
            alpha = x(44);
            multirotor.configController(attitudeController,P,Q,R,Ef,Eg,H,mu,alpha);
            index = 45;
        case 'RLQ-R Passive Modified'
            P = diag(x(13:18));
            Q = diag(x(19:24));
            R = diag(x(25:32));
            Ef = reshape(x(33:38),[1 6]);
            Eg = reshape(x(39:46),[1 8]);
            H = reshape(x(47:52),[6 1]);
            mu = x(53);
            alpha = x(54);
            multirotor.configController(attitudeController,P,Q,R,Ef,Eg,H,mu,alpha);
            index = 55;
        case 'RLQ-R Passive Modified with PIDD'
            P = diag(x(13:21));
            Q = diag(x(22:30));
            R = diag(x(31:38));
            Ef = reshape(x(39:47),[1 9]);
            Eg = reshape(x(48:55),[1 8]);
            H = reshape(x(56:64),[9 1]);
            mu = x(65);
            alpha = x(66);
            multirotor.configController(attitudeController,P,Q,R,Ef,Eg,H,mu,alpha);
            index = 67;
        case 'RLQ-R Active'
            P = diag(x(13:18));
            Q = diag(x(19:24));
            R = diag(x(25:27));
            Ef = reshape(x(28:33),[1 6]);
            Eg = reshape(x(34:36),[1 3]);
            H = reshape(x(37:42),[6 1]);
            mu = x(43);
            alpha = x(44);
            multirotor.configController(attitudeController,P,Q,R,Ef,Eg,H,mu,alpha);
            index = 45;
        case 'RLQ-R Active Modified'
            P = diag(x(13:18));
            Q = diag(x(19:24));
            R = diag(x(25:32));
            Ef = reshape(x(33:38),[1 6]);
            Eg = reshape(x(39:46),[1 8]);
            H = reshape(x(47:52),[6 1]);
            mu = x(53);
            alpha = x(54);
            multirotor.configController(attitudeController,P,Q,R,Ef,Eg,H,mu,alpha);
            index = 55;
        case 'RLQ-R Active Modified with PIDD'
            P = diag(x(13:21));
            Q = diag(x(22:30));
            R = diag(x(31:38));
            Ef = reshape(x(39:47),[1 9]);
            Eg = reshape(x(48:55),[1 8]);
            H = reshape(x(56:64),[9 1]);
            mu = x(65);
            alpha = x(66);
            multirotor.configController(attitudeController,P,Q,R,Ef,Eg,H,mu,alpha);
            index = 67;
        case 'SOSMC Passive'
            c = diag(x(13:15));
            alpha = diag(x(16:18));
            lambda = diag(x(19:21));
            multirotor.configController(attitudeController,c,lambda,alpha);
            index = 22;
        case 'SOSMC Passive with PIDD'
            c = diag(x(13:18));
            alpha = diag(x(19:24));
            lambda = diag(x(25:30));
            multirotor.configController(attitudeController,c,lambda,alpha);
            index = 31;
        case 'SOSMC Passive Direct'
            c = diag(x(13:18));
            alpha = reshape(x(19:66),[8 6]);   
            lambda = reshape(x(67:114),[8 6]);
            multirotor.configController(attitudeController,c,lambda,alpha,1,0);
            index = 115;
        case 'SOSMC Active'
            c = diag(x(13:15));
            alpha = diag(x(16:18));
            lambda = diag(x(19:21));
            multirotor.configController(attitudeController,c,lambda,alpha);
            index = 22;
        case 'SOSMC Active with PIDD'
            c = diag(x(13:18));
            alpha = diag(x(19:24));
            lambda = diag(x(25:30));
            multirotor.configController(attitudeController,c,lambda,alpha);
            index = 31;
        case 'SOSMC Active Direct'
            c = diag(x(13:18));
            alpha = reshape(x(19:66),[8 6]);   
            lambda = reshape(x(67:114),[8 6]);
            multirotor.configController(attitudeController,c,lambda,alpha);
            index = 115;
        case 'Adaptive'
            Am = diag(x(13:15));
            Q = diag(x(16:18));
            gamma1 = diag(x(19:21));
            gamma2 = diag(x(22:24));
            gamma3 = diag(x(25:27));
            gamma4 = diag(x(28:30));
            multirotor.configController(attitudeController,Am,Q,gamma1,gamma2,gamma3,gamma4);
            index = 31;
        case 'Adaptive with PIDD'
            Am = diag(x(13:18));
            Q = diag(x(19:24));
            gamma1 = diag(x(25:30));
            gamma2 = diag(x(31:36));
            gamma3 = diag(x(37:42));
            gamma4 = diag(x(43:48));
            multirotor.configController(attitudeController,Am,Q,gamma1,gamma2,gamma3,gamma4);
            index = 49;
        case 'Adaptive Direct'
            Am = diag(x(13:18));
            Q = diag(x(19:24));
            gamma1 = diag(x(25:32));
            gamma2 = diag(x(33:40));
            gamma3 = diag(x(41:48));
            gamma4 = diag(x(49:56));
            B0 = reshape(x(57:104),[8 6]);
            multirotor.configController(attitudeController,Am,Q,gamma1,gamma2,gamma3,gamma4,B0);
            index = 105;
        case 'Markovian RLQ-R Passive Modified'
            Ef = reshape(x(13:18),[1 6]);
            Eg = reshape(x(19:26),[1 8]);
            k = x(27);
            Er = diag(x(28:35));
            Eq = diag(x(36:41));
            lambda = x(42);
            modes = [1 1 1 1 1 1 1 1
                     0 1 1 1 1 1 1 1
                     0 0 1 1 1 1 1 1
                     0 0 0 1 1 1 1 1
                     0 0 0 0 1 1 1 1];
            numberOfModes = size(modes,1);
            pij = x(43)*eye(numberOfModes);
            eij = x(44)*ones(numberOfModes, numberOfModes);                 
            multirotor.configController(attitudeController,eye(6),Ef,Eg,k,Er,Eq,lambda,modes,pij,eij);
            index = 45;
    end
    multirotor.setAngularFilterGain(x(index:index+2));
    index = 108;
    switch controlAllocator
        case 'Adaptive'
            caGain = diag(x(index:index+5));
            multirotor.configControlAllocator(controlAllocator,caGain,1,0);
            index = 113;
    end
    
    multirotor.setController(attitudeController);
    multirotor.setControlAllocator(controlAllocator);
    multirotor.setAttitudeReferenceCA(attitudeReference);
end

