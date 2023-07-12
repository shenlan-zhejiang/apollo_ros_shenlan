"""Loads the osqp library"""

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
def clean_dep(dep):
    return str(Label(dep))

def repo():
    native.new_local_repository(
        name = "ompl",
        build_file = clean_dep("//third_party/ompl:ompl.BUILD"),
        path = "/apollo/third_party_shenlan/ompl/include/",
    )
