function diseno = diseno_pid(varargin)
%% ========================================================================
%  Wrapper retrocompatible hacia diseno-controlador-digital/disenar_pid.m
%  Mantiene intacta la API historica diseno_pid(...) usada por otros scripts.
% ========================================================================
rutaModulo = fullfile(fileparts(mfilename("fullpath")), "diseno-controlador-digital");
if isempty(which("disenar_pid"))
    addpath(rutaModulo);
end
diseno = disenar_pid(varargin{:});
end
