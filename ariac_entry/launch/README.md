# ECSE 373 Laboratory #5

## Set up

This package relies on the ARIAC 2019 environment which you can read more about [here] (https://bitbucket.org/osrf/ariac/wiki/2019/Home). 

Clone the following repositories and put them in their own separate workspaces:

<https://github.com/cwru-eecs-373/cwru_ariac_2019.git>

<https://github.com/cwru-eecs-373/ecse_373_ariac.git>

## Launching

`roslaunch ariac_entry entry.launch`

## About the package

The purpose of this package is to process orders in the ARIAC simulation by:

- Subscribing to the `/ariac/orders` topic to receive new orders which are pushed to a queue
- Using the `material_location` service to find the bin(s) that has a part of the type
required by the first product in the first shipment of the first order.
- Subscribing to all logical_cameras and storing the information
- Logging a message with the bin number and (x, y, Z) position of the part
