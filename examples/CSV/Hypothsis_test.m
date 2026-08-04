% =========================================================================
% Hypothsis test example
% =========================================================================
clear; clc; close all;
sim_length = 1000;
%% signal parameter
d_cfg.preamble_repeats  = 8;
d_cfg.preamble_len      = 256;
d_cfg.repeat_preamble_l = d_cfg.preamble_len * d_cfg.preamble_repeats;
d_cfg.SNR               = 5;
d_cfg.delay             = 1000;
d_cfg.N                 = d_cfg.preamble_len * (d_cfg.preamble_repeats - 1);

metric_store = zeros(2,sim_length);
corr_power   = zeros(2,sim_length);

%% Sim
for ii = 1:sim_length
    [metric_store(1,ii),corr_power(1,ii)] = H0_sim(d_cfg);
    [metric_store(2,ii),corr_power(2,ii)] = H1_sim(d_cfg);
end
%% plot and theory distribution
% figure
% hold on
% histogram(corr_power(1,:))
% histogram(corr_power(2,:))
% grid on
% title("Correlation Power")
% legend(["H0","H1"])

figure
eta = linspace(0,0.1,1000);
fm = 2*eta.*(d_cfg.N -1).*(1-eta.^2).^(d_cfg.N-2);
histogram(metric_store(1,:),'Normalization','pdf');
hold on;
plot(eta,fm,'LineWidth',2);
legend(["H0 in ","Beta PDF"])
grid on
title("H0 vs Beta PDF")


figure
histogram(metric_store(2,:),'Normalization','pdf')
grid on
legend("H1")
title("H1 Metric when SNR = "+num2str(d_cfg.SNR))

figure
hold on
histogram(metric_store(1,:),'Normalization','pdf')
histogram(metric_store(2,:),'Normalization','pdf')
plot(eta,fm,"b--",'LineWidth',2);
text = "H1 SNR = " + num2str(d_cfg.SNR);
legend(["H0",text,"Beta PDF"])
grid on
title("H0/H1 Metric")

function [metric,corr_power] = H0_sim(d_cfg)
    % signal parameter
    first_power     = 0;
    second_power    = 0;
    corr            = 0; 
    span            = d_cfg.preamble_len * (d_cfg.preamble_repeats - 1);
    w = (randn(1,d_cfg.repeat_preamble_l) + 1i*randn(1,d_cfg.repeat_preamble_l)) / sqrt(2);
    % Normaliztion

    % Correlation
    for i = 1:span
        first_power   = first_power + norm(w(i))^2;
        second_power  = second_power + norm(w(i + d_cfg.preamble_len))^2;
        corr          = corr + conj(w(i)) * (w(i + d_cfg.preamble_len));
    end
    corr_power = norm(corr)^2;
    metric = sqrt(corr_power/(first_power*second_power));
end

function [metric,corr_power] = H1_sim(d_cfg)
    % signal parameter
    first_power     = 0;
    second_power    = 0;
    corr            = 0; 
    span            = d_cfg.preamble_len * (d_cfg.preamble_repeats - 1);
    w = (randn(1,d_cfg.repeat_preamble_l) + 1i*randn(1,d_cfg.repeat_preamble_l)) / sqrt(2);
    seed            = uint32(13990001);
    % qpsk preambles
    Npilots      = d_cfg.preamble_len;
    mt_outputs   = std_mt19937_uint32(seed,Npilots);
    qpsk_pilots = complex(zeros(d_cfg.repeat_preamble_l, 1));
    qpsk_scale = 1 / sqrt(2);
    for ii = 1:Npilots
    
        bits = mt_outputs(ii);
    
        if bitand(bits, uint32(1)) ~= 0
            re = +qpsk_scale;
        else
            re = -qpsk_scale;
        end
    
        if bitand(bits, uint32(2)) ~= 0
            im = +qpsk_scale;
        else
            im = -qpsk_scale;
        end
    
        qpsk_pilots(ii) = complex(re, im);
    end
    qpsk_pilots = qpsk_pilots(1:d_cfg.preamble_len,1)';
    for ii = 1:log2(d_cfg.preamble_repeats)
        qpsk_pilots = [qpsk_pilots,qpsk_pilots];
    end
    % Normaliztion and SNR set
    Pw = norm(w)^2;
    Ps = norm(qpsk_pilots(1:span))^2;
    SNR_factor = sqrt(Pw/Ps*10^(d_cfg.SNR/10));

    qpsk_pilots = qpsk_pilots * SNR_factor;
    s = w + qpsk_pilots;
    % delay
    w_delay = (randn(1,d_cfg.repeat_preamble_l) + 1i*randn(1,d_cfg.repeat_preamble_l)) / sqrt(2);
    s = [w_delay(1:d_cfg.delay),s];

    % Correlation
    for i = 1:span
        first_power   = first_power + norm(s(i))^2;
        second_power  = second_power + norm(s(i + d_cfg.preamble_len))^2;
        corr          = corr + conj(s(i)) * (s(i + d_cfg.preamble_len));
    end
    corr_power = norm(corr)^2;
    metric = sqrt(corr_power/(first_power*second_power));
end



function output = std_mt19937_uint32(seed, count)

    N = 624;
    M = 397;

    MATRIX_A   = uint32(hex2dec('9908B0DF'));
    UPPER_MASK = uint32(hex2dec('80000000'));
    LOWER_MASK = uint32(hex2dec('7FFFFFFF'));

    state = zeros(N, 1, 'uint32');

    state(1) = uint32(seed);

    for i = 2:N

        previous = state(i - 1);

        z = bitxor(previous, bitshift(previous, -30));

        value64 = ...
            uint64(1812433253) * uint64(z) ...
            + uint64(i - 1);

        state(i) = uint32( ...
            bitand(value64, uint64(hex2dec('FFFFFFFF'))) ...
        );
    end

    index = N + 1;

    output = zeros(count, 1, 'uint32');

    for out_idx = 1:count

        if index > N

            for i = 1:N

                next_i = mod(i, N) + 1;

                y = bitor( ...
                    bitand(state(i), UPPER_MASK), ...
                    bitand(state(next_i), LOWER_MASK));

                source_i = mod(i - 1 + M, N) + 1;

                new_value = bitxor( ...
                    state(source_i), ...
                    bitshift(y, -1));

                if bitand(y, uint32(1)) ~= 0
                    new_value = bitxor(new_value, MATRIX_A);
                end

                state(i) = new_value;
            end

            index = 1;
        end

        y = state(index);
        index = index + 1;

        y = bitxor(y, bitshift(y, -11));

        y = bitxor( ...
            y, ...
            bitand(bitshift(y, 7), ...
                   uint32(hex2dec('9D2C5680'))));

        y = bitxor( ...
            y, ...
            bitand(bitshift(y, 15), ...
                   uint32(hex2dec('EFC60000'))));

        y = bitxor(y, bitshift(y, -18));

        output(out_idx) = y;
    end
end