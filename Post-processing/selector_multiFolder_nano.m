% function folderList = selector_multiFolder(startPath, dialogTitle)
% %SELECTOR_MULTIFOLDER Let user select multiple folders via UI.
% %
% %   folderList = selector_multiFolder()
% %   folderList = selector_multiFolder(startPath)
% %   folderList = selector_multiFolder(startPath, dialogTitle)
% %
% %   OUTPUT:
% %     folderList : cell array of selected folder paths (0x1 cell if none)
% 
%     % Defaults
%     if nargin < 1 || isempty(startPath)
%         startPath = pwd;
%     end
%     if nargin < 2 || isempty(dialogTitle)
%         dialogTitle = 'Select a folder (Cancel when done)';
%     end
% 
%     folderList = {};
% 
%     while true
%         folderName = uigetdir(startPath, dialogTitle);
% 
%         % User hit Cancel
%         if isequal(folderName, 0)
%             break;
%         end
% 
%         % Avoid duplicates
%         if ~any(strcmp(folderList, folderName))
%             folderList{end+1} = folderName; %#ok<AGROW>
%         end
% 
%         % Next iteration starts from last chosen folder
%         startPath = folderName;
%     end
% end

function folderList = selector_multiFolder(startPath, dialogTitle)
%SELECTOR_MULTIFOLDER Let user select multiple folders via a single UI dialog.
%
%   folderList = selector_multiFolder()
%   folderList = selector_multiFolder(startPath)
%   folderList = selector_multiFolder(startPath, dialogTitle)
%
%   OUTPUT:
%     folderList : cell array of selected folder paths (0x1 cell if none)
%
%   NOTE:
%     - Uses Java Swing (requires desktop MATLAB with Java enabled).
%     - Allows multi-selection of directories in one dialog.

    % Defaults
    if nargin < 1 || isempty(startPath)
        startPath = pwd;
    end
    if nargin < 2 || isempty(dialogTitle)
        dialogTitle = 'Select one or more folders';
    end

    % Ensure Java is available
    assert(usejava('desktop') || usejava('jvm'), ...
        'selector_multiFolder:JavaRequired', ...
        'Java-based UI is required for multi-folder selection.');

    import javax.swing.JFileChooser;
    import javax.swing.filechooser.FileSystemView;
    import java.io.File;

    % Set up file chooser for directories only, multi-select enabled
    fsv = FileSystemView.getFileSystemView();
    chooser = JFileChooser(fsv);

    chooser.setDialogTitle(dialogTitle);
    chooser.setFileSelectionMode(JFileChooser.DIRECTORIES_ONLY);
    chooser.setMultiSelectionEnabled(true);
    chooser.setAcceptAllFileFilterUsed(false);

    % Set initial directory
    if exist(startPath, 'dir')
        chooser.setCurrentDirectory(File(startPath));
    end

    % Show dialog (modal)
    status = chooser.showOpenDialog([]);

    if status == JFileChooser.APPROVE_OPTION
        files = chooser.getSelectedFiles();
        n = numel(files);
        folderList = cell(n, 1);
        for k = 1:n
            folderList{k} = char(files(k).getAbsolutePath());
        end
    else
        % User cancelled
        folderList = {};
    end
end