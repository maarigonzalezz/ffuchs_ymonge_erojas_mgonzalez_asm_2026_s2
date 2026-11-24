function T = leerCSV(path)
% Importacion explicita: un texto malformado se detecta como NaN, no se omite.
    if ~isfile(path)
        error("frenado:MissingFile", "No existe el CSV: %s", path);
    end
    opts = detectImportOptions(path, "VariableNamingRule", "preserve");
    names = string(opts.VariableNames);
    if ismember("phase", names)
        opts = setvartype(opts, "phase", "string");
        opts = setvartype(opts, cellstr(names(names ~= "phase")), "double");
    end
    T = frenado.validarTabla(readtable(path, opts));
end
