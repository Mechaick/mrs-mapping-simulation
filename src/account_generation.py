def generate_account_dict():
    from algosdk import account, mnemonic    
    private_key = account.generate_account()[0]    # need [0], because generate_account() returns a list    
    acc = {}
    acc['public'] = account.address_from_private_key(private_key)
    acc['private'] = private_key    
    acc['mnemonic'] = mnemonic.from_private_key(private_key)
    return acc

Turtle1 = generate_account_dict()
print(Turtle1)

Turtle2 = generate_account_dict()
print(Turtle2)