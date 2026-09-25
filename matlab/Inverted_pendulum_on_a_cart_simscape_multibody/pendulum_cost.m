function J = pendulum_cost(p, mdl)

% =========================================================
% PENDULUM_COST
%
% Költségfüggvény a fordított inga külső
% pozíciószabályozójának optimalizálásához.
%
% Az fminsearch változói:
%
%   p(1) = log(Kz)
%   p(2) = log(Kv)
%
% Emiatt:
%
%   Kz = exp(p(1))
%   Kv = exp(p(2))
%
% A cél:
%
%   1. kis kocsipozíció
%   2. kis dőlésszög
%   3. mérsékelt erő
%   4. z -> 0 a futás végére
%   5. phi -> 0 a futás végére
%   6. phi_ref ne verje állandóan a ±2 fokos limitet
%   7. phi_ref ne változzon túl gyorsan
%   8. a kezdeti tranziens után a tényleges phi
%      lehetőleg maradjon ±2 fokon belül
%
% =========================================================


    %% =====================================================
    %  1. OPTIMALIZÁLT PARAMÉTEREK
    % ======================================================

    Kz = exp(p(1));
    Kv = exp(p(2));


    %% =====================================================
    %  2. KERESÉSI TARTOMÁNY
    %
    % Az fminsearch önmagában nem korlátos optimalizáló.
    % Ezért az értelmetlen tartományokra óriási költséget
    % adunk.
    % ======================================================

    Kz_min = 1e-4;
    Kz_max = 0.05;

    Kv_min = 0.01;
    Kv_max = 5.0;


    if Kz < Kz_min || Kz > Kz_max
        J = 1e12;
        return;
    end


    if Kv < Kv_min || Kv > Kv_max
        J = 1e12;
        return;
    end


    %% =====================================================
    %  3. SIMULATION INPUT
    % ======================================================

    simIn = Simulink.SimulationInput(mdl);

    simIn = simIn.setVariable("Kz", Kz);
    simIn = simIn.setVariable("Kv", Kv);


    %% =====================================================
    %  4. SZIMULÁCIÓ
    % ======================================================

    try

        simOut = sim(simIn);


        %% =================================================
        %  5. LOGOLT JELEK
        % ==================================================

        z_ts       = simOut.z_log;
        phi_ts     = simOut.phi_log;
        F_ts       = simOut.F_log;
        phi_ref_ts = simOut.phi_ref_log;


        tz = z_ts.Time;
        z  = squeeze(z_ts.Data);


        tphi = phi_ts.Time;
        phi  = squeeze(phi_ts.Data);


        tF = F_ts.Time;
        F  = squeeze(F_ts.Data);


        tref = phi_ref_ts.Time;
        phi_ref = squeeze(phi_ref_ts.Data);


        %% =================================================
        %  6. ADATELLENŐRZÉS
        % ==================================================

        if isempty(z) || ...
           isempty(phi) || ...
           isempty(F) || ...
           isempty(phi_ref)

            J = 1e12;
            return;

        end


        if any(~isfinite(z)) || ...
           any(~isfinite(phi)) || ...
           any(~isfinite(F)) || ...
           any(~isfinite(phi_ref))

            J = 1e12;
            return;

        end


        %% =================================================
        %  7. NORMALIZÁLÓ ÉRTÉKEK
        %
        % Ezekkel az eltérő fizikai mennyiségeket
        % összehasonlítható nagyságrendbe hozzuk.
        % ==================================================

        z_scale = 0.05;             % 5 cm
        phi_scale = 2*pi/180;       % 2 fok
        F_scale = 10;               % 10 N


        zn = z / z_scale;

        phin = phi / phi_scale;

        Fn = F / F_scale;


        %% =================================================
        %  8. ALAP KÖLTSÉGEK
        %
        % A teljes idő alatt büntetjük:
        %
        %   z²
        %   phi²
        %   F²
        %
        % ==================================================

        J_z = trapz( ...
            tz, ...
            zn.^2);


        J_phi = trapz( ...
            tphi, ...
            phin.^2);


        J_F = trapz( ...
            tF, ...
            Fn.^2);


        %% =================================================
        %  9. ALAPKÖLTSÉG SÚLYAI
        %
        % A dőlésszög fontosabb, mint az erőfelhasználás.
        % ==================================================

        w_z = 1.0;

        w_phi = 5.0;

        w_F = 0.02;


        J_running = ...
              w_z   * J_z ...
            + w_phi * J_phi ...
            + w_F   * J_F;


        %% =================================================
        %  10. VÉGÁLLAPOT BÜNTETÉSE
        %
        % Nem elég az, hogy valamikor áthaladjon nullán.
        % A futás végén is ott akarjuk látni.
        % ==================================================

        z_final_normalized = z(end) / z_scale;

        phi_final_normalized = phi(end) / phi_scale;


        w_z_final = 50;

        w_phi_final = 50;


        J_terminal = ...
              w_z_final ...
            * z_final_normalized^2 ...
            + ...
              w_phi_final ...
            * phi_final_normalized^2;


        %% =================================================
        %  11. PHI_REF SZATURÁCIÓ BÜNTETÉSE
        %
        % Pos_limit = ±2 fok.
        %
        % Ha phi_ref szinte folyamatosan ±2 fokon ül,
        % az rossz, mert ez okozhatja a gyors kapcsolgatást.
        %
        % Már a limit 95%-ánál "szaturáltnak" tekintjük.
        % ==================================================

        phi_ref_limit = 2*pi/180;

        saturation_threshold = ...
            0.95 * phi_ref_limit;


        near_saturation = ...
            abs(phi_ref) >= saturation_threshold;


        J_phi_ref_sat = trapz( ...
            tref, ...
            double(near_saturation));


        w_phi_ref_sat = 20;


        J_phi_ref_sat = ...
            w_phi_ref_sat ...
            * J_phi_ref_sat;


        %% =================================================
        %  12. PHI_REF VÁLTOZÁSI SEBESSÉG BÜNTETÉSE
        %
        % Ez bünteti az ilyen viselkedést:
        %
        %   +2° -> -2° -> +2° -> -2°
        %
        % Minél gyorsabban változik phi_ref,
        % annál nagyobb a költség.
        % ==================================================

        if length(phi_ref) > 2

            dphi_ref = gradient( ...
                phi_ref, ...
                tref);

            J_phi_ref_rate = trapz( ...
                tref, ...
                dphi_ref.^2);

        else

            J_phi_ref_rate = 0;

        end


        w_phi_ref_rate = 2;


        J_phi_ref_rate = ...
            w_phi_ref_rate ...
            * J_phi_ref_rate;


        %% =================================================
        %  13. TÉNYLEGES PHI > ±2° BÜNTETÉSE
        %
        % Fontos:
        %
        % kezdetben phi = 5 fok körüli,
        % ezért az első 1 másodpercet nem büntetjük
        % ezen a kemény limiten keresztül.
        %
        % Utána viszont szeretnénk:
        %
        %       |phi| <= 2 fok
        %
        % ==================================================

        phi_limit = 2*pi/180;

        transient_time = 1.0;


        active = tphi >= transient_time;


        phi_excess = zeros(size(phi));


        phi_excess(active) = max( ...
            0, ...
            abs(phi(active)) - phi_limit);


        J_phi_limit = trapz( ...
            tphi, ...
            (phi_excess / phi_limit).^2);


        w_phi_limit = 100;


        J_phi_limit = ...
            w_phi_limit ...
            * J_phi_limit;


        %% =================================================
        %  14. KOCSI NAGY KITÉRÉSÉNEK BÜNTETÉSE
        %
        % 5 cm-en belül szeretnénk tartani.
        %
        % Ez nem hard constraint, hanem extra büntetés.
        % ==================================================

        z_limit = 0.05;


        z_excess = max( ...
            0, ...
            abs(z) - z_limit);


        J_z_limit = trapz( ...
            tz, ...
            (z_excess / z_limit).^2);


        w_z_limit = 20;


        J_z_limit = ...
            w_z_limit ...
            * J_z_limit;


        %% =================================================
        %  15. TÚL NAGY ABSZOLÚT KITÉRÉS
        %
        % Biztonsági büntetés.
        % ==================================================

        J_emergency = 0;


        if max(abs(z)) > 1.0

            J_emergency = ...
                J_emergency + 1e5;

        end


        if max(abs(phi)) > 30*pi/180

            J_emergency = ...
                J_emergency + 1e6;

        end


        %% =================================================
        %  16. TELJES KÖLTSÉG
        % ==================================================

        J = ...
              J_running ...
            + J_terminal ...
            + J_phi_ref_sat ...
            + J_phi_ref_rate ...
            + J_phi_limit ...
            + J_z_limit ...
            + J_emergency;


        %% =================================================
        %  17. DEBUG INFORMÁCIÓ
        % ==================================================

        max_z = max(abs(z));

        max_phi_deg = ...
            max(abs(phi))*180/pi;


        sat_percentage = ...
            100 ...
            * mean(near_saturation);


        fprintf( ...
            ['Kz=%9.6f | ' ...
             'Kv=%8.4f | ' ...
             'J=%10.3f | ' ...
             'zmax=%7.4f m | ' ...
             'phimax=%6.2f deg | ' ...
             'sat=%5.1f %%\n'], ...
            Kz, ...
            Kv, ...
            J, ...
            max_z, ...
            max_phi_deg, ...
            sat_percentage);


        %% =================================================
        %  18. RÉSZKÖLTSÉGEK KIÍRÁSA
        %
        % Így látod, MIÉRT adott ekkora J-t.
        % ==================================================

        fprintf( ...
            ['   Jz=%8.2f | ' ...
             'Jphi=%8.2f | ' ...
             'JF=%8.2f | ' ...
             'Jterm=%8.2f | ' ...
             'Jsat=%8.2f | ' ...
             'Jrate=%8.2f | ' ...
             'JphiLim=%8.2f | ' ...
             'JzLim=%8.2f\n'], ...
            w_z * J_z, ...
            w_phi * J_phi, ...
            w_F * J_F, ...
            J_terminal, ...
            J_phi_ref_sat, ...
            J_phi_ref_rate, ...
            J_phi_limit, ...
            J_z_limit);


    catch ME

        fprintf( ...
            ['Szimulacios hiba ' ...
             '[Kz=%.6g Kv=%.6g]: %s\n'], ...
            Kz, ...
            Kv, ...
            ME.message);


        J = 1e12;

    end

end