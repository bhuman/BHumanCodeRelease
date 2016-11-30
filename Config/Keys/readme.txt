use private key file to log into nao.

example:
$ ssh -i $PATH_TO_KEY/id_rsa_nao -l root 134.102.204.229

modify ~/.ssh/config:
|Host nao
|       HostName 134.102.204.229   (changed by dhcp)
|	User root
|	IdentityFile ~/.ssh/id_rsa_nao

