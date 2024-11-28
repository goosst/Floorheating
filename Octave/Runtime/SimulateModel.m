function state_new=SimulateModel(modeldefinition,intg,parameters,controlinputs,state)

  res = intg('x0',state,'u',controlinputs,'p',parameters);
  state_new = full(res.xf);

end
