function [statistic, selected_corners] = ORB_binned_detector (left_rectified_img,...
    ORB_params, binning_params)
    % Generate arrays of bin ROI
    total_ROI = ORB_params.ROI;
    num_pixel_row = total_ROI(4);
    num_pixel_col = total_ROI(3);

    step_col = floor(num_pixel_col/binning_params.num_col);
    step_row = floor(num_pixel_row/binning_params.num_row);
    selected_corners.Location = [];
    
    for rowrow = 1:binning_params.num_row
        bin_start_row = (rowrow - 1) * step_row + total_ROI(2); 
        bin_height = min(num_pixel_row - bin_start_row + total_ROI(2),step_row);
        for colcol = 1:binning_params.num_col
            bin_start_col  = (colcol - 1) * step_col + total_ROI(1);
            bin_width = min(num_pixel_col - bin_start_col + total_ROI(1), step_col);
            cur_bin_ROI = [bin_start_col, bin_start_row, bin_width, bin_height];
            ORB_corners = detectORBFeatures(left_rectified_img, ...
                'ScaleFactor',ORB_params.scale_factor, 'NumLevels', ORB_params.num_level,...
                'ROI', cur_bin_ROI);
            if ORB_corners.Count >= binning_params.num_point_per_bin
                [~,sort_ind] = sort(ORB_corners.Metric, 'descend');
                raw_loc = round(ORB_corners.Location);
                selected_corners.Location = vertcat(selected_corners.Location, raw_loc(sort_ind(1:binning_params.num_point_per_bin,:),:));
            end
        end
    end

    statistic.num_keypoint_detector = size(selected_corners.Location,1);
end