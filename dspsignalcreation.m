clc; clear; close all;

%% ========================= USER SETTINGS =========================
portName   = "COM4";
baudRate   = 115200;
timeoutSec = 10;
modeStr    = "Q31";          

% --- Dynamic Signal Settings ---
Fs = 2000;           
N  = 256;            

% We vary the frequencies slightly each time to change the data bits
f1 = 125 + (randi(10) - 5); % Varies between 120-130Hz
f2 = 300 + (randi(20) - 10);
f3 = 500 + (randi(30) - 15);

a1 = 0.78; a2 = 0.36; a3 = 0.24;
queryStatusAfterRun = true;
%% ================================================================

fprintf('================ STM32 DYNAMIC FFT ================\n');
fprintf('Target Frequencies: %.1f, %.1f, %.1f Hz\n', f1, f2, f3);

%% Build time-domain signal
t = (0:N-1) / Fs;
% Added a bit of random noise to ensure the CPU cycles vary slightly
x = a1*sin(2*pi*f1*t) + a2*sin(2*pi*f2*t) + a3*sin(2*pi*f3*t) + 0.05*randn(size(t));
x = x(:).'; 

%% Prepare serial port
cleanupSerial(portName);
s = serialport(portName, baudRate, "Timeout", timeoutSec);
configureTerminator(s, "LF");
pause(0.5); 
flush(s);

%% Select STM32 Mode
writeline(s, "mode " + lower(modeStr));
modeReply = readNonEmptyLine(s);
disp("STM32: " + modeReply);

%% Trigger Run
writeline(s, "run");
runReply = readNonEmptyLine(s);
disp("STM32: " + runReply);

if ~contains(runReply, "READY_BIN")
    error("STM32 not ready.");
end

%% Send Samples
% CRITICAL: Flush right before sending to prevent byte-shift ($10^{34}$)
pause(0.1);
flush(s); 

if strcmpi(modeStr, "F32")
    write(s, single(x), "single");
else
    q31Scale = 2^31;
    x_q31 = int32(round(min(max(x, -1.0), 1-1/q31Scale) * q31Scale));
    write(s, x_q31, "int32");
end

%% Read Magnitudes back
numMag = N/2 + 1;
mag_raw = read(s, numMag, "single");
mag_stm32 = double(mag_raw);

%% Query Status (This is where you see the cycles/time)
if queryStatusAfterRun
    pause(0.2);
    % Clear the buffer one more time to make sure we aren't reading 
    % leftover binary data as text
    flush(s, "input");
    writeline(s, "status");
    statusText = readStatusBlock(s);
    fprintf('\n--- UPDATED STM32 PERFORMANCE ---\n%s\n', statusText);
    fftTimeUs = parseLastFftUs(statusText);
end

%% Plotting
%% 1. Calculate MATLAB Reference & Absolute Error
X_ref = fft(x, N);
mag_matlab = abs(X_ref(1:N/2+1)) / N;
mag_matlab(2:end-1) = 2 * mag_matlab(2:end-1); % Match STM32 scaling
f_axis = (0:N/2) * Fs / N;

% Calculate Absolute Error
absErr = abs(mag_stm32 - mag_matlab);

%% 2. Generate Combined Dashboard
figure('Name', 'STM32 FFT System Performance', 'Color', 'w', 'Units', 'normalized', 'Position', [0.1 0.1 0.6 0.8]);

% --- Subplot 1: Time Domain ---
subplot(3,1,1);
plot(t, x, 'LineWidth', 1.2, 'Color', [0.2 0.2 0.2]);
grid on;
title(['Input Signal: ', num2str(f1), 'Hz, ', num2str(f2), 'Hz, ', num2str(f3), 'Hz']);
xlabel('Time (s)'); ylabel('Amplitude');

% --- Subplot 2: FFT Comparison ---
subplot(3,1,2);
stem(f_axis, mag_stm32, 'filled', 'MarkerFaceColor', [0 0.447 0.741]);
hold on;
plot(f_axis, mag_matlab, 'r--', 'LineWidth', 1.5);
grid on;
title(['FFT Magnitude (STM32 Execution Time: ', num2str(fftTimeUs), ' us)']);
xlabel('Frequency (Hz)'); ylabel('Magnitude');
legend('STM32 (F32)', 'MATLAB (64-bit)');

% --- Subplot 3: Absolute Error ---
subplot(3,1,3);
plot(f_axis, absErr, 'r', 'LineWidth', 1.2);
grid on;
title('Absolute Error: |STM32 - MATLAB|');
xlabel('Frequency (Hz)'); ylabel('Error');
ylim([0 max(absErr)*1.2 + 1e-7]); % Auto-scale with a small floor

fprintf('Dashboard updated with Absolute Error. Max Error: %.4e\n', max(absErr));
%% ========================= LOCAL FUNCTIONS =========================
function cleanupSerial(portName)
    oldObj = serialportfind("Port", portName);
    if ~isempty(oldObj)
        delete(oldObj);
    end
end

function line = readNonEmptyLine(s)
    line = ""; t0 = tic;
    while strlength(line) == 0 && toc(t0) < s.Timeout
        if s.NumBytesAvailable > 0
            line = strtrim(string(readline(s)));
        end
    end
end

function statusText = readStatusBlock(s)
    lines = strings(0); started = false; t0 = tic;
    while toc(t0) < s.Timeout
        if s.NumBytesAvailable > 0
            line = strtrim(string(readline(s)));
            if contains(line, "STATUS BEGIN"), started = true; end
            if started, lines(end+1) = line; end
            if contains(line, "STATUS END"), break; end
        end
    end
    statusText = strjoin(lines, newline);
end

function fftTimeUs = parseLastFftUs(statusText)
    fftTimeUs = NaN;
    token = regexp(statusText, '(\d+)\s*us', 'tokens', 'once');
    if ~isempty(token), fftTimeUs = str2double(token{1}); end
end