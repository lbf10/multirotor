% Generate latex variables for best individuals
clear all
filelist = dir(uigetdir);
filelist(1:2) = [];

saveDir = uigetdir('','Choose a folder to save files to');

for it=1:length(filelist)
    if ~filelist(it).isdir
        clear params
        filename = [filelist(it).folder,'/',filelist(it).name]
        data = load(filename);
        [pathname,name,extension] = fileparts(filename);
        % Verifica se é arquivo finalizado de simulação ou não e retira melhor
        % indivíduo da otimização
        if any(strcmp(fieldnames(data),'bestIndividual'))
            bestIndividual = data.bestIndividual;
            bestScore = data.bestFitness;
        else
            scores = data.state(end).Score;
            population = data.state(end).Population;
            [bestScore,bestIndex] = min(scores);
            bestIndividual = population(bestIndex,:);
        end
        controllerConfig = strsplit(name,'_');
        attitudeController = controllerConfig{1};
        controlAllocator = controllerConfig{2};
        attitudeReference = controllerConfig{3};
        
        x = bestIndividual;
        %UNTITLED Summary of this function goes here
        %   Detailed explanation goes here
        params.K_P = x(1:3);
        params.K_I = x(4:6);
        params.K_D = x(7:9);
        params.K_DD = x(10:12);
        switch attitudeController
            case 'PID'
                params.K_tauP = x(13:15);
                params.K_tauI = x(16:18);
                params.K_tauD = x(19:21);
                index = 22;
            case 'RLQ-R Passive'
                params.P = diag(x(13:18));
                params.Q = diag(x(19:24));
                params.R = diag(x(25:27));
                params.E_f = reshape(x(28:33),[1 6]);
                params.E_g = reshape(x(34:36),[1 3]);
                params.H = reshape(x(37:42),[6 1]);
                params.mu = x(43);
                params.alpha = x(44);
                index = 45;
            case 'RLQ-R Passive Modified'
                params.P = diag(x(13:18));
                params.Q = diag(x(19:24));
                params.R = diag(x(25:32));
                params.E_f = reshape(x(33:38),[1 6]);
                params.E_g = reshape(x(39:46),[1 8]);
                params.H = reshape(x(47:52),[6 1]);
                params.mu = x(53);
                params.alpha = x(54);
                index = 55;
            case 'RLQ-R Passive Modified with PIDD'
                params.P = diag(x(13:21));
                params.Q = diag(x(22:30));
                params.R = diag(x(31:38));
                params.E_f = reshape(x(39:47),[1 9]);
                params.E_g = reshape(x(48:55),[1 8]);
                params.H = reshape(x(56:64),[9 1]);
                params.mu = x(65);
                params.alpha = x(66);
                index = 67;
            case 'RLQ-R Active'
                params.P = diag(x(13:18));
                params.Q = diag(x(19:24));
                params.R = diag(x(25:27));
                params.E_f = reshape(x(28:33),[1 6]);
                params.E_g = reshape(x(34:36),[1 3]);
                params.H = reshape(x(37:42),[6 1]);
                params.mu = x(43);
                params.alpha = x(44);
                index = 45;
            case 'RLQ-R Active Modified'
                params.P = diag(x(13:18));
                params.Q = diag(x(19:24));
                params.R = diag(x(25:32));
                params.E_f = reshape(x(33:38),[1 6]);
                params.E_g = reshape(x(39:46),[1 8]);
                params.H = reshape(x(47:52),[6 1]);
                params.mu = x(53);
                params.alpha = x(54);
                index = 55;
            case 'RLQ-R Active Modified with PIDD'
                params.P = diag(x(13:21));
                params.Q = diag(x(22:30));
                params.R = diag(x(31:38));
                params.E_f = reshape(x(39:47),[1 9]);
                params.E_g = reshape(x(48:55),[1 8]);
                params.H = reshape(x(56:64),[9 1]);
                params.mu = x(65);
                params.alpha = x(66);
                index = 67;
            case 'SOSMC Passive'
                params.c = diag(x(13:15));
                params.lambda = diag(x(16:18));
                params.alpha = diag(x(19:21));
                index = 22;
            case 'SOSMC Passive with PIDD'
                params.c = diag(x(13:18));
                params.lambda = diag(x(19:24));
                params.alpha = diag(x(25:30));
                index = 31;
            case 'SOSMC Passive Direct'
                params.c = diag(x(13:18));
                params.lambda = reshape(x(19:66),[8 6]);
                params.alpha = reshape(x(67:114),[8 6]);
                index = 115;
            case 'SOSMC Active'
                params.c = diag(x(13:15));
                params.lambda = diag(x(16:18));
                params.alpha = diag(x(19:21));
                index = 22;
            case 'SOSMC Active with PIDD'
                params.c = diag(x(13:18));
                params.lambda = diag(x(19:24));
                params.alpha = diag(x(25:30));
                index = 31;
            case 'SOSMC Active Direct'
                params.c = diag(x(13:18));
                params.lambda = reshape(x(19:66),[8 6]);
                params.alpha = reshape(x(67:114),[8 6]);
                index = 115;
            case 'Adaptive'
                params.Am = diag(x(13:15));
                params.Q = diag(x(16:18));
                params.gamma_1 = diag(x(19:21));
                params.gamma_2 = diag(x(22:24));
                params.gamma_3 = diag(x(25:27));
                params.gamma_4 = diag(x(28:30));
                index = 31;
            case 'Adaptive with PIDD'
                params.Am = diag(x(13:18));
                params.Q = diag(x(19:24));
                params.gamma_1 = diag(x(25:30));
                params.gamma_2 = diag(x(31:36));
                params.gamma_3 = diag(x(37:42));
                params.gamma_4 = diag(x(43:48));
                index = 49;
            case 'Adaptive Direct'
                params.Am = diag(x(13:18));
                params.Q = diag(x(19:24));
                params.gamma_1 = diag(x(25:32));
                params.gamma_2 = diag(x(33:40));
                params.gamma_3 = diag(x(41:48));
                params.gamma_4 = diag(x(49:56));
                params.B_0 = reshape(x(57:104),[8 6]);
                index = 105;
            case 'Markovian RLQ-R Passive Modified'
                Ef = reshape(x(13:42),[1 6 5]);
                params.E_f1 = Ef(:,:,1);
                params.E_f2 = Ef(:,:,2);
                params.E_f3 = Ef(:,:,3);
                params.E_f4 = Ef(:,:,4);
                params.E_f5 = Ef(:,:,5);
                Eg = reshape(x(43:82),[1 8 5]);
                params.E_g1 = Eg(:,:,1);
                params.E_g2 = Eg(:,:,2);
                params.E_g3 = Eg(:,:,3);
                params.E_g4 = Eg(:,:,4);
                params.E_g5 = Eg(:,:,5);
                params.k = x(83);
                params.E_r1 = diag(x(84:91));
                params.E_r2 = diag(x(92:99));
                params.E_r3 = diag(x(100:107));
                params.E_r4 = diag(x(108:115));
                params.E_r5 = diag(x(116:123));
                params.E_q1 = diag(x(124:129));
                params.E_q2 = diag(x(130:135));
                params.E_q3 = diag(x(136:141));
                params.E_q4 = diag(x(142:147));
                params.E_q5 = diag(x(148:153));
                params.lambda = x(154);
                params.modes = [1 1 1 1 1 1 1 1
                    0 1 1 1 1 1 1 1
                    0 0 1 1 1 1 1 1
                    0 0 0 1 1 1 1 1
                    0 0 0 0 1 1 1 1];
                params.numberOfModes = size(params.modes,1);
                params.p_ij = x(155)*eye(params.numberOfModes);
                params.e_ij = x(156)*ones(params.numberOfModes, params.numberOfModes);
                index = 157;
            case 'Markovian RLQ-R Active Modified'
                params.modes = [1 1 1 1 1 1 1 1
                    0 1 1 1 1 1 1 1
                    0 0 1 1 1 1 1 1
                    0 0 0 1 1 1 1 1
                    0 0 0 0 1 1 1 1];
                params.numberOfModes = size(params.modes,1);
                params.P = eye(6);
                params.Q_1 = diag(x(13:18));
                params.Q_2 = diag(x(19:24));
                params.Q_3 = diag(x(25:30));
                params.Q_4 = diag(x(31:36));
                params.Q_5 = diag(x(37:42));
                params.R_1 = diag(x(43:50));
                params.R_2 = diag(x(51:58));
                params.R_3 = diag(x(59:66));
                params.R_4 = diag(x(67:74));
                params.R_5 = diag(x(75:82));
                Ef = reshape(x(83:112),[1 6 5]);
                params.E_f1 = Ef(:,:,1);
                params.E_f2 = Ef(:,:,2);
                params.E_f3 = Ef(:,:,3);
                params.E_f4 = Ef(:,:,4);
                params.E_f5 = Ef(:,:,5);
                Eg = reshape(x(113:152),[1 8 5]);
                params.E_g1 = Eg(:,:,1);
                params.E_g2 = Eg(:,:,2);
                params.E_g3 = Eg(:,:,3);
                params.E_g4 = Eg(:,:,4);
                params.E_g5 = Eg(:,:,5);
                H = reshape(x(153:182),[6 1 5]);
                params.H_1 = H(:,:,1);
                params.H_2 = H(:,:,2);
                params.H_3 = H(:,:,3);
                params.H_4 = H(:,:,4);
                params.H_5 = H(:,:,5);
                params.p_ij = x(183)*eye(params.numberOfModes);
                params.e_i = x(184)*ones(1,params.numberOfModes);
                params.k = x(185);
                params.mu = x(186);
                params.alpha = x(187);
                index = 188;
        end
        params.angularFilterGain = x(index:index+2);
        fileID = fopen([saveDir,'/bestIndividualParams_',name,'.txt'],'w');
        fieldNames = fields(params);
        for jt =1:length(fieldNames)
            fprintf(fileID,'\n$ %s = %s $\n',fieldNames{jt},latex(vpa(params.(fieldNames{jt}),4)));
        end
        fclose(fileID);
    end
end

