function img = decodeRos2ImageEcho(logPath, outputPath)
%DECODEROS2IMAGEECHO Decode a sensor_msgs/Image saved from `ros2 topic echo`.
%
%   img = decodeRos2ImageEcho(logPath)
%   img = decodeRos2ImageEcho(logPath, outputPath)
%
% ROS sensor_msgs/Image stores height as rows, width as columns, and data as
% row-major bytes. For rgb8/bgr8 images the bytes are interleaved per pixel:
%   row 1: R G B, R G B, ...
%
% If the echo has an empty encoding but step == width * 3, this function
% treats it as RGB interleaved because that matches the generated Jetson code.

    if nargin < 1 || strlength(string(logPath)) == 0
        [fileName, fileDir] = uigetfile({'*.txt;*.log;*.yaml;*.yml', ...
            'ROS2 echo logs (*.txt, *.log, *.yaml, *.yml)'; '*.*', 'All files'});
        if isequal(fileName, 0)
            error('No input file selected.');
        end
        logPath = fullfile(fileDir, fileName);
    end

    if nargin < 2
        outputPath = "";
    end

    txt = fileread(logPath);

    height = readScalarField(txt, "height");
    width = readScalarField(txt, "width");
    step = readScalarField(txt, "step");
    encoding = readStringField(txt, "encoding");

    dataStart = regexp(txt, "(?m)^data:\s*$", "end", "once");
    if isempty(dataStart)
        error('Could not find a "data:" block in %s.', logPath);
    end

    dataTxt = txt(dataStart + 1:end);
    values = regexp(dataTxt, "(?m)^\s*-\s*(\d+)\s*$", "tokens");
    if isempty(values)
        error('Found "data:" but no byte entries of the form "- 123".');
    end

    data = uint8(str2double(string([values{:}])));
    expectedBytes = double(height) * double(step);
    if numel(data) < expectedBytes
        error('Image data is incomplete: expected %d bytes, found %d.', ...
            expectedBytes, numel(data));
    elseif numel(data) > expectedBytes
        warning('Ignoring %d extra byte entries after the first image.', ...
            numel(data) - expectedBytes);
        data = data(1:expectedBytes);
    end

    channels = inferChannelCount(encoding, width, step);
    rowBytes = double(width) * channels;
    if double(step) < rowBytes
        error('Step (%d) is too small for width %d with %d channels.', ...
            step, width, channels);
    end

    rows = reshape(data, double(step), double(height)).';
    rows = rows(:, 1:rowBytes);

    if channels == 1
        img = reshape(rows.', double(width), double(height)).';
    else
        img = permute(reshape(rows.', channels, double(width), double(height)), ...
            [3 2 1]);
    end

    if strcmpi(encoding, "bgr8")
        img = img(:, :, [3 2 1]);
    end

    figure;
    imshow(img);
    title(sprintf('%s: %dx%d, %d channel(s)', string(encoding), width, height, channels), ...
        'Interpreter', 'none');

    if strlength(string(outputPath)) > 0
        imwrite(img, outputPath);
        fprintf('Wrote decoded image to %s\n', outputPath);
    end
end

function value = readScalarField(txt, fieldName)
    pattern = "(?m)^" + fieldName + ":\s*(\d+)\s*$";
    token = regexp(txt, pattern, "tokens", "once");
    if isempty(token)
        error('Could not find scalar field "%s".', fieldName);
    end
    value = uint32(str2double(token{1}));
end

function value = readStringField(txt, fieldName)
    pattern = "(?m)^" + fieldName + ":\s*['""]?([^'""\r\n]*)['""]?\s*$";
    token = regexp(txt, pattern, "tokens", "once");
    if isempty(token)
        value = "";
    else
        value = strtrim(string(token{1}));
    end
end

function channels = inferChannelCount(encoding, width, step)
    enc = lower(string(encoding));

    if any(enc == ["mono8", "8uc1"])
        channels = 1;
    elseif any(enc == ["rgb8", "bgr8", "8uc3"])
        channels = 3;
    elseif any(enc == ["rgba8", "bgra8", "8uc4"])
        channels = 4;
    elseif step == width
        channels = 1;
    elseif step == width * 3
        channels = 3;
    elseif step == width * 4
        channels = 4;
    else
        error(['Cannot infer channel count from encoding "%s", width %d, ' ...
            'and step %d.'], encoding, width, step);
    end
end
