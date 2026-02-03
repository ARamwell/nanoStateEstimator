function img_out = overlayPoseOnImage(img_in, poseVec)
%OVERLAYPOSEONIMAGE Superimpose pose array values on the image.

    %img_vis = fliplr(rot90(img_in, -1)); % flip for display so overlays aren't mirrored
    img_vis = img_in;
    if ndims(img_vis) == 2
        img_working = cat(3, img_vis, img_vis, img_vis);
    else
        img_working = img_vis;
    end

    yPos = 10;
    lineSpacing = 28;
    
    %poseVec = poseArr(:, s);
    if ~isnan(poseVec(1))
        label = sprintf('Pose: [%.2f %.2f %.2f | %.2f %.2f %.2f %.2f]', ...
                        poseVec(1), poseVec(2), poseVec(3), ...
                        poseVec(4), poseVec(5), poseVec(6), poseVec(7));
        img_working = insertText(img_working, [10 yPos], label, ...
                                 'FontSize', 18, ...,
                                 'TextColor', [1 1 1]);
        yPos = yPos + lineSpacing;
    end

    %mark (1,1)

    img_out = img_working;
    %img_out = rot90(fliplr(img_working));
end
