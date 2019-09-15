function plotVec(origin, target, opts)
%     quiver3(origin(1), origin(2), origin(3), ...
%         vec(1), vec(2), vec(3));
    if size(origin) == [3 1]
        origin = origin';
    end
    if size(target) == [3 1]
        target = target';
    end
    
    vec = [target; origin];

    plot3(vec(:, 1), vec(:, 2), vec(:, 3), opts);
    grid on;
end