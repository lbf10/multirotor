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
            lambda = diag(x(16:18));
            alpha = diag(x(19:21));
            multirotor.configController(attitudeController,c,lambda,alpha);
            index = 22;
        case 'SOSMC Passive with PIDD'
            c = diag(x(13:18));
            lambda = diag(x(19:24));
            alpha = diag(x(25:30));
            multirotor.configController(attitudeController,c,lambda,alpha);
            index = 31;
        case 'SOSMC Passive Direct'
            c = diag(x(13:18));
            lambda = reshape(x(19:66),[8 6]);   
            alpha = reshape(x(67:114),[8 6]);
            multirotor.configController(attitudeController,c,lambda,alpha,1,0);
            index = 115;
        case 'SOSMC Active'
            c = diag(x(13:15));
            lambda = diag(x(16:18));
            alpha = diag(x(19:21));
            multirotor.configController(attitudeController,c,lambda,alpha);
            index = 22;
        case 'SOSMC Active with PIDD'
            c = diag(x(13:18));
            lambda = diag(x(19:24));
            alpha = diag(x(25:30));
            multirotor.configController(attitudeController,c,lambda,alpha);
            index = 31;
        case 'SOSMC Active Direct'
            c = diag(x(13:18));
            lambda = reshape(x(19:66),[8 6]);   
            alpha = reshape(x(67:114),[8 6]);
            multirotor.configController(attitudeController,c,lambda,alpha,1,0);
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
            Ef = reshape(x(13:42),[1 6 5]);
            Eg = reshape(x(43:82),[1 8 5]);
            k = x(83);
            Er = [];
            Er(:,:,1) = diag(x(84:91));
            Er(:,:,2) = diag(x(92:99));
            Er(:,:,3) = diag(x(100:107));
            Er(:,:,4) = diag(x(108:115));
            Er(:,:,5) = diag(x(116:123));
            Eq = [];
            Eq(:,:,1) = diag(x(124:129));
            Eq(:,:,2) = diag(x(130:135));
            Eq(:,:,3) = diag(x(136:141));
            Eq(:,:,4) = diag(x(142:147));
            Eq(:,:,5) = diag(x(148:153));
            lambda = x(154);
            modes = [1 1 1 1 1 1 1 1
                     0 1 1 1 1 1 1 1
                     0 0 1 1 1 1 1 1
                     0 0 0 1 1 1 1 1
                     0 0 0 0 1 1 1 1];
            numberOfModes = size(modes,1);
            pij = x(155)*eye(numberOfModes);
            eij = x(156)*ones(numberOfModes, numberOfModes);                 
            multirotor.configController(attitudeController,modes,eye(6),Ef,Eg,k,Er,Eq,lambda,pij,eij);
            index = 157;
        case 'Markovian RLQ-R Active Modified'            
            modes = [1 1 1 1 1 1 1 1
                     0 1 1 1 1 1 1 1
                     0 0 1 1 1 1 1 1
                     0 0 0 1 1 1 1 1
                     0 0 0 0 1 1 1 1];
            numberOfModes = size(modes,1);
            P = [];
            for it = 1:numberOfModes
                  P(:,:,it) = eye(6);  
            end
            Q = [];
            Q(:,:,1) = diag(x(13:18));
            Q(:,:,2) = diag(x(19:24));
            Q(:,:,3) = diag(x(25:30));
            Q(:,:,4) = diag(x(31:36));
            Q(:,:,5) = diag(x(37:42));
            R = [];
            R(:,:,1) = diag(x(43:50));
            R(:,:,2) = diag(x(51:58));
            R(:,:,3) = diag(x(59:66));
            R(:,:,4) = diag(x(67:74));
            R(:,:,5) = diag(x(75:82));
            Ef = reshape(x(83:112),[1 6 5]);
            Eg = reshape(x(113:152),[1 8 5]);
            H = [];
            H = reshape(x(153:182),[6 1 5]);
            pij = x(183)*eye(numberOfModes);
            ei = x(184)*ones(1,numberOfModes);
            k = x(185);
            mu = x(186);
            alpha = x(187);               
            multirotor.configController(attitudeController,modes,P,Q,R,Ef,Eg,H,pij,ei,k,mu,alpha);
            index = 188;
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

