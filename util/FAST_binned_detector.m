function [statistic, selected_corners] = FAST_binned_detector (left_rectified_img,...
    FAST_params, binning_params)
    % Generate arrays of bin ROI
    total_ROI = FAST_params.ROI;
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
            num_keypoints = 0;
            cur_qual = FAST_params.min_quality_initial_guess;
            while num_keypoints < binning_params.num_point_per_bin
                if cur_qual < 0
                    break
                end
                FAST_corners = detectFASTFeatures(left_rectified_img, ...
                    'MinQuality',cur_qual, 'ROI', cur_bin_ROI,'MinContrast',FAST_params.bin_min_contrast);
                num_keypoints = FAST_corners.Count;
                cur_qual = cur_qual - 0.001;    
            end
            if FAST_corners.Count >= binning_params.num_point_per_bin
                [~,sort_ind] = sort(FAST_corners.Metric, 'descend');
                raw_loc = FAST_corners.Location;
                selected_corners.Location = vertcat(selected_corners.Location, raw_loc(sort_ind(1:binning_params.num_point_per_bin,:),:));
            end
        end
    end

    statistic.num_keypoint_detector = size(selected_corners.Location,1);

    
end