int main() {

    using Obs = Observable<std::string, std::string>;

    auto obs1 = std::make_shared< Obs >();
    auto obs2 = std::make_shared< Obs >();
    auto obs2 = std::make_shared< Obs >();

    auto dfg = std::make_shared<DataFlowGraph>();

    AnyComputation comp1; 
    comp1.f = [=]() {
        std::cout << "Parent val: " << obs2.value() << std::endl;
    };
    comp1.requires_waiting_on_ros = true;

    dfg.add_v(obs1);
    dfg.add_v(obs2);
    dfg.add_v(obs3);

    dfg.add_edge(obs3->index.value(), obs2->index.value(), {comp1});

    GraphEngine graph_engine(dfg);

    graph_engine.register_trigger(obs1);
    graph_engine.run();


    obs1->resolve("hello");
    /// Should stop 

    /// Now propagate further
    obs2->resolve("hello obs2");

}
