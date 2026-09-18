function fileList = selector_multiFile(startPath, dialogTitle, multiSelect, filterSpec)
%SELECTOR_MULTIFILE Let user select one or more files via a single UI dialog.
%
%   fileList = selector_multiFile()
%   fileList = selector_multiFile(startPath)
%   fileList = selector_multiFile(startPath, dialogTitle)
%   fileList = selector_multiFile(startPath, dialogTitle, multiSelect)
%   fileList = selector_multiFile(startPath, dialogTitle, multiSelect, filterSpec)
%
%   INPUT:
%     startPath   : initial folder (default: pwd)
%     dialogTitle : dialog title (default: 'Select one or more files')
%     multiSelect : logical, allow multi-select (default: true)
%     filterSpec  : file pattern(s), e.g. '*.mat' or {'*.mat','*.log'} (default: all)
%
%   OUTPUT:
%     fileList : cell array of selected file paths (0x1 cell if none)

    if nargin < 1 || isempty(startPath),   startPath   = pwd; end
    if nargin < 2 || isempty(dialogTitle), dialogTitle = 'Select one or more files'; end
    if nargin < 3 || isempty(multiSelect), multiSelect = true; end
    if nargin < 4, filterSpec = []; end

    assert(usejava('desktop') || usejava('jvm'), ...
        'selector_multiFile:JavaRequired', ...
        'Java-based UI is required for multi-file selection.');

    import javax.swing.JFileChooser;
    import javax.swing.filechooser.FileSystemView;
    import javax.swing.filechooser.FileNameExtensionFilter;
    import java.io.File;

    fsv = FileSystemView.getFileSystemView();
    chooser = JFileChooser(fsv);

    chooser.setDialogTitle(dialogTitle);
    chooser.setFileSelectionMode(JFileChooser.FILES_ONLY);
    chooser.setMultiSelectionEnabled(logical(multiSelect));
    chooser.setAcceptAllFileFilterUsed(isempty(filterSpec));

    % Apply filter if requested
    if ~isempty(filterSpec)
        if ischar(filterSpec)
            filterSpec = {filterSpec};
        end
        % Extract extensions without '*.' and build filter
        exts = cellfun(@(s) regexprep(s, '^\*\.', ''), filterSpec, 'UniformOutput', false);
        desc = strjoin(filterSpec, ', ');
        filter = FileNameExtensionFilter(desc, exts{:});
        chooser.setFileFilter(filter);
    end

    % Initial directory
    if exist(startPath, 'dir')
        chooser.setCurrentDirectory(File(startPath));
    elseif exist(fileparts(startPath), 'dir')
        chooser.setCurrentDirectory(File(fileparts(startPath)));
    end

    status = chooser.showOpenDialog([]);

    if status == JFileChooser.APPROVE_OPTION
        if chooser.isMultiSelectionEnabled()
            files = chooser.getSelectedFiles();
            n = numel(files);
            fileList = cell(n, 1);
            for k = 1:n
                fileList{k} = char(files(k).getAbsolutePath());
            end
        else
            f = chooser.getSelectedFile();
            if isempty(f)
                fileList = {};
            else
                fileList = {char(f.getAbsolutePath())};
            end
        end
    else
        fileList = {};
    end
end