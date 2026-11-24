function T = leerCSV(path)
% Importacion explicita para esquemas step (9 cols) y runpid (15 cols).
% Un texto malformado en columnas numericas se detecta como NaN, no se omite.
    if ~isfile(path)
        error("frenado:MissingFile", "No existe el CSV: %s", path);
    end
    opts = detectImportOptions(path, "VariableNamingRule", "preserve");
    names = string(opts.VariableNames);
    textCols = intersect(names, ["phase", "ctrl_mode"], "stable");
    numCols = setdiff(names, textCols, "stable");
    if ~isempty(textCols)
        opts = setvartype(opts, cellstr(textCols), "string");
    end
    if ~isempty(numCols)
        opts = setvartype(opts, cellstr(numCols), "double");
    end
    T = frenado.validarTabla(readtable(path, opts));
end
