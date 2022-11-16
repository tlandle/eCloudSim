# Ansible Docs

### Summary

Ansible is used to provision our infrastructure in Azure. Specifically it will ssh into each one of our VMs and run the modules within the ansible playbooks.

### Inventory

Our inventory file is located in inventory/myazure_rm.yml. This is a dynamic inventory which means that it will grab a list of the current VMs we have in our azure resource group.


### Variables

The vars file in vm_management/vars/vars.yml is used by the notebooks in vm_management. We use the vars file to specify vm hosts and the resource group.

### Usage

#### To start all VMS:

```ansible-playbook ./vm_management/azure-vm-start.yml```


#### To provision all VMs:

```ansible-playbook -i ./inventory/myazure_rm.yml azure-vm-provision.yml```


#### Run Simulation on all VMs:

```ansible-playbook -i ./inventory/myazure_rm.yml azure-vm-simstart.yml```


#### Stop and delete containers on all VMs:

```ansible-playbook -i ./inventory/myazure_rm.yml azure-vm-container-cleanup.yml```

#### Delete docker images on all VMs:

```ansible-playbook -i ./inventory/myazure_rm.yml azure-vm-image-cleanup.yml```

#### To stop all VMS:

```ansible-playbook ./vm_management/azure-vm-stop.yml```

