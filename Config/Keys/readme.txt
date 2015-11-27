use private key file to log into nao.

example:
$ ssh -i $PATH_TO_KEY/id_rsa_nao -l root 169.254.220.70

modify ~/.ssh/config:
|Host nao
|       HostName 169.254.220.70   (changed by dhcp)
|	User root
|	IdentityFile ~/.ssh/id_rsa_nao

