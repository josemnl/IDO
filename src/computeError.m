function [error, falsePositiveError, falseNegativeError] = computeError(ground_truth, inference, cx, ci, radius)
    n_cells = 0;
    total_error = 0;
    n_cells_f_pos = 0;
    total_error_f_pos = 0;
    n_cells_f_neg = 0;
    total_error_f_neg = 0;
    for i = 1:size(ground_truth,1)
        for j = 1:size(ground_truth,2)
            if (i-cx)^2 + (j-ci)^2 <= radius^2
                n_cells = n_cells + 1;
                total_error = total_error + abs(ground_truth(i,j)-inference(i,j));
                if ground_truth(i,j) == 0
                    n_cells_f_pos = n_cells_f_pos + 1;
                    total_error_f_pos = total_error_f_pos + abs(ground_truth(i,j)-inference(i,j));
                else
                    n_cells_f_neg = n_cells_f_neg + 1;
                    total_error_f_neg = total_error_f_neg + abs(ground_truth(i,j)-inference(i,j));
                end
            end
        end
    end
    falsePositiveError = total_error_f_pos/n_cells_f_pos;
    falseNegativeError = total_error_f_neg/n_cells_f_neg;
    error = total_error/n_cells;
end