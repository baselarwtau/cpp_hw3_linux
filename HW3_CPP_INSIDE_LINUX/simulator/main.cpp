int main() {
    // dlopen
    for(const auto& algo: AlgorithmRegistrar::getAlgorithmRegistrar()) {
        auto algorithm = algo.create();
        std::cout << algo.name() << ": " << static_cast<int>(algorithm->nextStep()) << std::endl;
    }
    AlgorithmRegistrar::getAlgorithmRegistrar().clear();
    // dlclose
}
