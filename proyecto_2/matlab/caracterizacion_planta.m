function modelo = caracterizacion_planta(varargin)
%% ========================================================================
%  Wrapper retrocompatible hacia caracterizacion_parametros/caracterizar_planta.m
%  Mantiene intacta la API historica caracterizacion_planta(...) usada por
%  tests/test_frenado.m, identificacion_semiautomatica.m y graficar_ensayos.py.
% ========================================================================
rutaModulo = fullfile(fileparts(mfilename("fullpath")), "caracterizacion_parametros");
if isempty(which("caracterizar_planta"))
    addpath(rutaModulo);
end
modelo = caracterizar_planta(varargin{:});
end
