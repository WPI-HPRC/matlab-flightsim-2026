% One day when I am sick of Matlab, we are exporting CSV files to Python
% and building the stuff there

%% Convert N×3 table variables to CSV
mat_file = load("dynamics.mat");
myTable = struct2table(mat_file.ans);
T = myTable;

vars = T.Properties.VariableNames;

Tout = table();

for i = 1:numel(vars)

    name = vars{i};
    data = T.(name).Data;
    if ndims(data) == 3
        data = permute(data, [3,1,2]);
    end

    disp(name)

    if isnumeric(data) && ndims(data) == 3 && size(data,2) == 3
        Tout.([name '_00']) = data(:,1,1);
        Tout.([name '_01']) = data(:,1,2);
        Tout.([name '_02']) = data(:,1,3);
        Tout.([name '_10']) = data(:,2,1);
        Tout.([name '_11']) = data(:,2,2);
        Tout.([name '_12']) = data(:,2,3);
        Tout.([name '_20']) = data(:,3,1);
        Tout.([name '_21']) = data(:,3,2);
        Tout.([name '_22']) = data(:,3,3);



    % If this is an N×3 numeric array
    elseif isnumeric(data) && size(data,2) == 3

        Tout.([name '_0']) = data(:,1);
        Tout.([name '_1']) = data(:,2);
        Tout.([name '_2']) = data(:,3);

    elseif isnumeric(data) && size(data,2) == 4

        Tout.([name '_0']) = data(:,1);
        Tout.([name '_1']) = data(:,2);
        Tout.([name '_2']) = data(:,3);
        Tout.([name '_3']) = data(:,4);


    % If this is an N×1 numeric array
    elseif isnumeric(data) && size(data,2) == 1

        Tout.(name) = data;

    else
        warning('Skipping %s (class: %s, size: %s)', ...
            name, class(data), mat2str(size(data)));
    end
end

%% Write CSV
writetable(Tout, 'dynamics.csv');

