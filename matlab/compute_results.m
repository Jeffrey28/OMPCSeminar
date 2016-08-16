clear all;

allRes = {};
allRes.ltv = zeros(4, 4, 3, 3); % errors(y-rms,y-max,psi-rms,psi-max), velocities, horizons, mus
allRes.nlp = zeros(4, 4, 3, 3);
muNames = {'_mu03', '_mu06', '_mu1'};
vNames = {'_v7', '_v10', '_v15', '_v20'};
hNames = {'Hp7_Hc3', 'Hp15_Hc10', 'Hp25_Hc15'};

for muIdx = 1:3
    for hIdx = 1:3
        for vIdx = 1:4
            
            fName = char(strcat(hNames(hIdx), vNames(vIdx), muNames(muIdx), '.mat'));
            disp(fName);
            load(fName);
            
            % NLP
            [Y_rms, Y_max] = YRMS(res_nlp);
            [Psi_rms, Psi_max] = PsiRMS(res_nlp);
            
            allRes.nlp(1, vIdx, hIdx, muIdx) = Y_rms;
            allRes.nlp(2, vIdx, hIdx, muIdx) = Y_max;
            allRes.nlp(3, vIdx, hIdx, muIdx) = Psi_rms;
            allRes.nlp(4, vIdx, hIdx, muIdx) = Psi_max;
            
            % LTV
            [Y_rms, Y_max] = YRMS(res_ltv);
            [Psi_rms, Psi_max] = PsiRMS(res_ltv);
            
            allRes.ltv(1, vIdx, hIdx, muIdx) = Y_rms;
            allRes.ltv(2, vIdx, hIdx, muIdx) = Y_max;
            allRes.ltv(3, vIdx, hIdx, muIdx) = Psi_rms;
            allRes.ltv(4, vIdx, hIdx, muIdx) = Psi_max;
        end
    end
end

%% Plot stuff
muNames = {'\mu = 0.3', '\mu = 0.6', '\mu = 1'};
errorNames = {'Y_{rms} [m]', 'Y_{max} [m]', '\psi_{rms} [\circ]', '\psi_{max} [\circ]'};
vNames = {'v=7m/s', 'v=10m/s', 'v=15m/s', 'v=20m/s'};
hNames = {'7 / 3', '15 / 10', '25 / 15'};

nlpRes = allRes.nlp;
ltvRes = allRes.ltv;
range = 1:3;

figure;
for errIdx = 1:4
    for muIdx = 1:3
        subplot(4, 3, muIdx + 3 * (errIdx-1));
        
        semilogy(range, squeeze(nlpRes(errIdx, 1, :, muIdx)), '-or');
        hold on;
        semilogy(range, squeeze(nlpRes(errIdx, 2, :, muIdx)), '-og');
        semilogy(range, squeeze(nlpRes(errIdx, 3, :, muIdx)), '-ob');
        semilogy(range, squeeze(nlpRes(errIdx, 4, :, muIdx)), '-om');

        semilogy(range, squeeze(ltvRes(errIdx, 1, :, muIdx)), '-.or');
        semilogy(range, squeeze(ltvRes(errIdx, 2, :, muIdx)), '-.og');
        semilogy(range, squeeze(ltvRes(errIdx, 3, :, muIdx)), '-.ob');
        semilogy(range, squeeze(ltvRes(errIdx, 4, :, muIdx)), '-.om');
        hold off;
        grid on;
        if errIdx == 1
            title(char(muNames(muIdx)));
            if muIdx == 1
                %legend(2, 'NL-MPC v=7 m/s','NL-MPC v=10 m/s','NL-MPC v=15 m/s','NL-MPC v=20 m/s','LTV-MPC v=7 m/s', 'LTV-MPC v=10 m/s', 'LTV-MPC v=15 m/s', 'LTV-MPC v=20 m/s', 'Orientation','horizontal');
            end
        end
        
        if errIdx == 4
            xlabel('Horizon (H_p / H_c)');
        end
        
        if muIdx == 1
            ylabel(char(errorNames(errIdx)));
        end
        
        set(gca,'XTickLabel',{'7 / 3', '15 / 10', '25 / 15'});
        set(gca,'XTick',[1:3]);
        set(gca,'XLim',[.9 3.1]);
        
        switch errIdx
            case 1
                set(gca,'YLim',[1e-5 1e5]);
            case 2
                set(gca,'YLim',[1e-2 1e2]);
            case 3
                set(gca,'YLim',[1e-2 1e5]);
            case 4
                set(gca,'YLim',[1e-1 1e3]);
        end
    end
end













