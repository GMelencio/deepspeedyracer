## Hyperparams

Batch:
Collecting batch size of experience after completition, sample set smaller is more stabile.
Number of epoch:

Learning rate:
Size of update after each training. Smaller size more stable model. Carefull to not divergent.
Entropy:
Randomness in data. Exploration vs Exploitation. Trying out new things.
Discount factor:
How many steps ahead is looking into. 0.999 default, 1000 steps ahead.
Can slower down the lap.
Loss type:
Predictions vs truth. Default Huber loss. Huber makes smaller increments, than mean squared error.  Updates model  during the process of back propagation.
Number of episodes  between policy updates: 
how many episodes should  agent run in between model updates.
more episodes in more experience. more complex it should be more.

Keep training models short interval and clone them.
Record detailed notes on current snapshats.
Use the logs.
More aggressive hyperparams on start, and later fine tune it .
Larger batch size, more learning rate, more episodes ---aggresive.
