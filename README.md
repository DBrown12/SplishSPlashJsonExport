This is a modification to the SplishSPlash branch that includes our Json Exporter, our modifications to rigid bodies. This is also our repo for talking about our project, we have uploaded our original simulation code for generating training data before finding SplishSplash. Along with all of our .Json files that have been generated with SplishSplash to train our NN. We've included our Feed Forward NN code as well. Due to the scale of each time step being above 2 gigs, we realized that our current approach with a Forward First NN wouldn't be possible, as one simulation would take over 500 hours of training time. It would have scalability issues, and at that point might as well just be done with our first itteration. But this was the approach:

Created our own Simulation Tool before finding SplishSplash:
Created to generate particle data that we used to interpolate a flow vector field. 
We quickly realized we'd want to have more data if we were planning to train a NN.
After this we found SplishSPlash

Feed Forward NN

1. Particle data is used to interpolate a flow vector field.
2. Then reduce the dimensions of the flow vector fields using PCA
3. Train on the PCA representation with a Feed-Forward NN. The Feed-Forward NN is trained to predict the next time-step of the flow vector field based positions of rigid bodies in the flow.
4. Use predicted flow vector field to simulate particle movements via numerical integration.
5. Take the trained model and bring that into a real time engine like UE5 or Unity and see how it works.

As previously stated due to the issue of scalability with large time steps because of a dimensionality problem. Along with the fact that sparse particle data results in unphysical vector fields, and the high likelihood that we run into severe numerical integration error down the line.

This lead us down a path looking at Lagrangian Graph Neural Network (LGNN) https://github.com/ravinderbhattoo/LGNN. We've been looking at ways to use this to deal with our data issue when loading large time steps. We believe we have a pretty decent idea oh how to handle this going forward. Along with believing that We'll be able to simulate large timesteps with less stability loss.

