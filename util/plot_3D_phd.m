function plot_3D_phd (map_est, transparency)
    sqrt_cov = map_est.max_likeli_gm_cov .^ 0.5;
    hold on
    for ii = 1:size(map_est.max_likeli_gm_inten,2)
        inten = map_est.max_likeli_gm_inten(1,ii)/2;

        phd_mu_x = map_est.max_likeli_gm_mu(1,ii);
        phd_mu_y = map_est.max_likeli_gm_mu(2,ii);
        phd_mu_z = map_est.max_likeli_gm_mu(3,ii);

        phd_std_x = sqrt_cov(1,1,ii);
        phd_std_y = sqrt_cov(2,2,ii);
        phd_std_z = sqrt_cov(3,3,ii);

        [X,Y,Z] = ellipsoid(phd_mu_x, phd_mu_y, phd_mu_z, phd_std_x, phd_std_y, phd_std_z, 10);
        C = ones(size(Z)) * inten;
        surf(X,Y,Z,C,'EdgeColor','none','FaceColor','flat','FaceAlpha',transparency)
        clim([0,2])
    end
end