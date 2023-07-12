"""Loads the osqp library"""

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
def clean_dep(dep):
    return str(Label(dep))

def repo():
    native.new_local_repository(
        name = "osqp2",
        build_file = clean_dep("//third_party/osqp2:osqp2.BUILD"),
        path = "/apollo/third_party_shenlan/osqp/include/",
    )
