function plot_3D_phd (map_est, sigma_mult, transparency)
    hold on
    for ii = 1:size(map_est.max_likeli_gm_inten,2)
        inten = map_est.max_likeli_gm_inten(1,ii)/2;

        h = plot_gaussian_ellipsoid(map_est.max_likeli_gm_mu(:,ii),...
            map_est.max_likeli_gm_cov(:,:,ii)*sigma_mult, inten);
        set(h,'facealpha',transparency,'EdgeColor','none','FaceColor','flat')
        clim([0,1.4])
    end
end