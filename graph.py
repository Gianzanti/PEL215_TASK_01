# import seaborn as sns
# import numpy as np

# sns.set_theme()

# with np.load("p_0_0_i_0_0_d_0_0.npz") as f:
#     data_setPoint = (f["setPoint"],)
#     data_position = (f["position"],)
#     data_error = (f["error"],)
#     data_rightWheelSpeed = (f["rightWheelSpeed"],)

# sns.relplot(data=data_setPoint, kind="line").set(
#     xlim=(0, None), ylim=(0, None), xlabel="Time step", ylabel="v"
# )


# # test_array = np.random.rand(3, 2)
# # test_vector = np.random.rand(4)
# # np.savez_compressed('/tmp/123', a=test_array, b=test_vector)
# # loaded = np.load('/tmp/123.npz')
# # print(np.array_equal(test_array, loaded['a']))
# # True
# # print(np.array_equal(test_vector, loaded['b']))
# # True


# Import seaborn
import seaborn as sns

# Apply the default theme
sns.set_theme()

# Load an example dataset
tips = sns.load_dataset("tips")

# Create a visualization
sns.relplot(
    data=tips,
    x="total_bill",
    y="tip",
    col="time",
    hue="smoker",
    style="smoker",
    size="size",
)
