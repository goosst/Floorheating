function state_new=SimulateModel(modeldefinition,intg,parameters,controlinputs,state)

%  intg=integrator('intg','cvodes',modeldefinition,0,Ts);
  res = intg('x0',state,'u',controlinputs,'p',parameters);
  state_new = res.xf;

end
